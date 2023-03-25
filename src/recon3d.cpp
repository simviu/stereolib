/*
   Author: Sherman Chen
   Create Time: 2023-01-20
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"
#include "json/json.h"
#include "vsn/vsnLibCv.h"

using namespace stereo;

namespace{
    struct LCfg{
        float fps = 30; 
    }; LCfg lc_;
    
}
//---- util
namespace{
    //--------
    float fp16_to_float(uint16_t x)
    {
        unsigned sign = ((x >> 15) & 1);
        unsigned exponent = ((x >> 10) & 0x1f);
        unsigned mantissa = ((x & 0x3ff) << 13);
        if (exponent == 0x1f) {  /* NaN or Inf */
            mantissa = (mantissa ? (sign = 0, 0x7fffff) : 0);
            exponent = 0xff;
        } else if (!exponent) {  /* Denorm or Zero */
            if (mantissa) {
                unsigned int msb;
                exponent = 0x71;
                do {
                    msb = (mantissa & 0x400000);
                    mantissa <<= 1;  /* normalize */
                    --exponent;
                } while (!msb);
                mantissa &= 0x7fffff;  /* 1.mantissa is implicit */
            }
        } else {
            exponent += 0x70;
        }
        uint32_t temp = ((sign << 31) | (exponent << 23) | mantissa);

        return *((float*)((void*)&temp));
    }
    
}

//----------
// ReconFrm
//----------
// Factory
Sp<Recon3d::Frm> Recon3d::Frm::create()
{
    return mkSp<ReconFrm>();
}
//-------
bool ReconFrm::calc(const Recon3d::Cfg& cfg)
{
    bool ok = true;

    //--- undistortion map(rectify)
    ok &= rectify(cfg.cams);

    //---- calc disparity
    auto& uds = data_.ud_imgs;
    assert(uds.size()>1);
    ok &= depth.calc_dispar(cfg.disp, *uds[0], *uds[1]);
    if(!ok) return false;
    //--- recon
    ok &= recon(cfg);
    return true;
}


//------------------------------
bool Recon3d::Cfg::load(const string& sf)
{

    log_i("Load Recon3d cfg :'"+sf+"'");
    ifstream ifs(sf);
    sys::FPath fp(sf);

    if(!ifs)
    {
        log_ef(sf);
        return false;
    }
    bool ok = true;
    //----
    try{

        Json::Reader rdr;
        Json::Value jd;
        rdr.parse(ifs, jd);
       
        string sfc = fp.path + jd["cams_cfg"].asString();
        if(!cams.load(sfc)) 
            return false;
        //---
        {
            frms.sDirs.clear();
            auto jf = jd["frms"];
            for(auto& j : jf["labels"])
                frms.sDirs.push_back(j.asString());
            frms.color_img = std::stoi(jf["color_img"].asString());
            frms.dispar_img = std::stoi(jf["dispar_img"].asString());
            frms.depth_img = std::stoi(jf["depth_img"].asString());
        }    
        //---
        string sfd = jd.get("disparity", "").asString();
        if(sfd!="")
            ok &= disp.load(fp.path + sfd);
        //--- DepthC
        {
            auto jt = jd.get("depth", "");
            string sr = jt.get("range", "").asString();
            vector<double> ds; 
            if(s2data(sr, ds)) depth.range = {ds[0], ds[1]}; 
            else ok = false;
        }
        
    }
    catch(exception& e)
    {
        log_e("exception caught:"+string(e.what()));
        return false;
    }
    
    if(!ok)
    {
        log_e("CamsCfg::load() json error");
        return false;
    }    
    //---- after process
    visc.pntvc.axisL = depth.range.d1;
    //visc.pntvc.axisL = 0.1; // dbg
    //----
    log_i("Recon3d cfg loaded '"+sf +"'");
    return true;
}
//-----

bool Recon3d::Frm::load_imgs(const Cfg& cfg, const string& sPath, int i)
{
    string si = to_string(i);
    auto& sDirs = cfg.frms.sDirs;
    int N = sDirs.size();
    int k=0;
    for(k=0;k<N;k++)
    {
        string sdir = sDirs[k];
        /*
        int flag = (k==cfg.frms.color_img) ?  cv::IMREAD_COLOR :
                   (k==cfg.frms.depth_img) ?  cv::IMREAD_ANYDEPTH :
                   cv::IMREAD_GRAYSCALE;
                   */
        int flag = -1 ; // unchange
        auto p = Img::loadFile(sPath + "/"+sdir+"/"+si+".png", flag);        
        if(p==nullptr) break;
        imgs.push_back(p);
        //--- dbg
        //int tp = p->type();
        //log_d("  type:"+to_string(tp));
    }
    if(k<N)
    {
        log_e("not all img loaded OK in path:'"+sPath+"'");
        return false;
    }
    //--- convert color img to CV_8UC3
    /*
    int ic = cfg.frms.color_img;
    if(ic>=0)
    {
        assert(ic<imgs.size());
        auto p = imgs[ic];
        cv::Mat imc0 = img2cv(*p);
        // TODO: convertTo can't handle channel differs
        cv::Mat imc1; imc0.convertTo(imc1, CV_8UC3);
        int tp0 = imc0.type(); // dbg
        int tp1 = imc1.type(); // dbg
        p = mkSp<ImgCv>(imc1);
        imgs[ic] = p;

    }
    */
    return true;

}
//----
bool Recon3d::Frm::load(const Cfg& cfg, const string& sPath, int i)
{
    bool ok = true;
    ok &= load_imgs(cfg, sPath, i);
    return ok;
}
//----
bool Recon3d::Frm::rectify(const CamsCfg& camcs)
{
  //  auto& ccvd = cast_imp(*camcs.get_cvd());
    int N = camcs.cams.size();
    assert(N<=imgs.size());
  //  assert(camcs.cams.size()>imgs.size());
  //  assert(camcs.cams.size()==ccvd.remapds.size());
    auto& cvd = *camcs.get_cvd();

    for(int i=0;i<N;i++)
    {
        //auto pu = cvd.remap(*imgs[i], i);  
        auto& cc = camcs.cams[i].camc; 
        auto pu = cc.undist(*imgs[i]);
        data_.ud_imgs.push_back(pu);  
    }
    return true;
}

//----
bool Recon3d::Frm::genPnts(const Cfg& cfg)
{
    if(cfg.frms.depth_img>=0)
        return genPnts_byDepth(cfg);
    else if(cfg.frms.dispar_img>=0)
        return genPnts_byDisp(cfg);
    else // Full pipeline L/R stereo from scratch
        return genPnts_byLR(cfg);
}
//----
bool Recon3d::Frm::genPnts_byDepth(const Cfg& cfg)
{
    int i_d = cfg.frms.depth_img;
    assert(i_d<imgs.size());
    auto pd = imgs[i_d];
    assert(pd!=nullptr);
    

    // assume depth aligned with RGB
    int i_c = cfg.frms.color_img;
    assert(i_c<imgs.size());
    auto pc = imgs[i_c];

    auto& cc_c = cfg.cams.cams[i_c].camc;
    Sz sz = cc_c.sz;
    //pd->scale(sz);
    ImgCv imcc(*pc);
    ImgCv imcd(*pd);
    cv::Mat imc = imcc.im_;
    cv::Mat imd = imcd.im_;
    cv::resize(imd, imd, cv::Size(sz.w, sz.h), cv::INTER_NEAREST);
    int tpd = imd.type();
    cv::Mat K; cv::eigen2cv(cc_c.K, K);
    for(int i=0;i<imc.rows;i++)
        for(int j=0;j<imc.cols;j++)
        {
            auto& dx = imd.ptr<uint16_t>(i)[j];
        //    double d = fp16_to_float(dx);
            double d = dx;
            double z = d*0.001; // was mm
            auto c = imc.ptr<const BGR>(i)[j];
            if(dx!=0)
            { int dbg=0;}
            Points::Pnt p;
            p.c = {c.r,c.g,c.b,255};
            vec2 px; px << j,i;
            p.p = cc_c.proj(px, z);
            pnts.add(p);
        }
    return true;
}
//----
bool Recon3d::Frm::genPnts_byLR(const Cfg& cfg)
{
    // img 0/1 are always L/R
    assert(imgs.size()>1);
    bool ok = true;

    //-----
    auto p_imd = depth.p_im_disp;
    assert(p_imd);
    cv::Mat imd = img2cv(*p_imd);
    int tp = imd.type();

  //calc_disp_to_pnts_cv(cfg, imd, pnts);
    disp_to_pnts(cfg);
    
    log_d("gen_pnts: "+pnts.info());
    
    return true;
}
//----
bool Recon3d::Frm::genPnts_byDisp(const Cfg& cfg)
{

    //--- if depth image is disparity
    /*
    cv::Mat Q; // Q from stereoRectify
    cv::eigen2cv(cfg.cams.rectify.Q, Q);
    Q.convertTo(Q, CV_64FC1);
    int tpQ = Q.type();
    cv::Mat im3d;
    cv::Mat imd; imcd.im_.convertTo(imd, CV_8UC1);

    int tpd = imd.type();
    cv::reprojectImageTo3D(imd, im3d, Q);
    int tp3 = im3d.type();
    */
    //----
    log_e("not yet");
    return false;
}
//----
/*
bool Recon3d::Frm::renderPnts(const Cfg& cfg)
{
    int ic = cfg.frms.color_img; 
    if(ic<0) return false; // no color
    assert(ic < imgs.size());
    auto p_imc = imgs[ic];
    for(auto& p : pnts.getData())
    {

    }
    return true;
}
*/
//-----
bool Recon3d::Frm::recon(const Cfg& cfg)
{
    bool ok = true;
    ok &= genPnts(cfg);
    return true;

}


//---------------
// calc_disp_to_pnts
//---------------
void Recon3d::Frm::disp_to_pnts(const Cfg& cfg)
{
    //----
    auto& ccs = cfg.cams.cams;
    assert(ccs.size()>1);
    auto& cc0 = ccs[0].camc; // Left cam
    double b = ccs[1].T.t.norm(); // baseline
    CamCfg::Lense l; cc0.toLense(l);
    double fx = l.fx; // focal length
    //---- get disparity and color
    int ic = cfg.frms.color_img;
    assert(ic < imgs.size()); 
    auto p_imd = depth.p_im_disp;
    assert(p_imd!=nullptr);
    auto imd = img2cv(*p_imd);
    Sp<Img> p_imc = nullptr; // color img may not have
    if(ic>=0) p_imc = imgs[ic];
    //auto imc = img2cv(*p_imc);
    auto& imc = *p_imc;
    //int tp1 = imc.type();// dbg
    //assert(tp1 == CV_8UC4);
    //----
    pnts.clear();
    int k=0;
    int tp = imd.type(); // dbg
    for(unsigned int y = 0; y < imd.rows; y++)
    {
        float* prow = (float*)imd.ptr<CV_32F>(y);
        for(unsigned int x = 0; x < imd.cols; x++)
        {
            double d = prow[x]; // disparity
            if(d <=0)continue;
            if(std::isnan(d)||std::isinf(d))
                continue;

            Points::Pnt p;
            //--- calc location
            double z = b * fx / d;
            vec2 q; q << x, y;
            vec3 v = cc0.proj(q, z);
            p.p = v;
            //--- get color
            p.c = {255,255,255,255};
            if(p_imc!=nullptr)
            {
                assert(ic < ccs.size());
                auto& c1 = ccs[ic];
                auto& camc = c1.camc;
                auto& T = c1.T;
                auto sz = p_imc->size();
                // transform to color camera frame.
                vec3 v1 = T * v;  
                vec2 q1 = camc.proj(v1);
                Px px = toPx(q1);

                if(!sz.isIn(px))continue;
                //BGRA c = imc.ptr<BGRA>(px.y)[px.x];
                //p.c = c.toUt();
                Color c; imc.get(px, c);
                p.c = c;
            }
            //---
            pnts.add(p);
        }

    }
}
//----------
// Recon3d
//----------
//----
void Recon3d::init_cmds()
{
    sHelp_ = "(Recon 3d point cloud from frms)";


    Cmd::add("init", mkSp<Cmd>("cfg=<CFG_FILE>",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        return cfg_.load(lookup(kv, "cfg")); 
    }));

    Cmd::add("frms", mkSp<Cmd>("dir=<DIR> (Run frms)",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        string sdir = lookup(kv, "dir"); 
        return run_frms(sdir); 
    }));

    Cmd::add("frm", mkSp<Cmd>("dir=<DIR> i=<IDX> (Run one frm)",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        string sdir = lookup(kv, "dir"); 
        int i=-1; s2d(lookup(kv, "i"), i); 
        if(i<0) return false;
        return run_frm(sdir, i); 
    }));
}



//---
bool Recon3d::onImg(Frm& f)
{
    
    //---- recon
    bool ok = true;
    ok &= f.calc(cfg_);

    //--- show
    show(f);
    return true;

}
//----
bool Recon3d::run_frm(const string& sPath, int i)
{

    log_i("frm:"+str(i));
    auto p = Frm::create();
    if(!p->load(cfg_, sPath, i))
        return false;
    
    //---- call
    bool ok = onImg(*p);
    if(!ok)
    {
        log_e("Recon3d::run_frm() failed");
        return false;
    }
    
    //----
    auto pv = get_frm_pnt_vis();
    assert(pv!=nullptr);
    while(ok)
    {
        pv->spin();
        cv_waitESC(5);
        sys::sleep(1.0/lc_.fps);    
    }
    return ok;
}

//----
bool Recon3d::run_frms(const string& sPath)
{
    //----
    int i=0;
    while(1)
    {
        i++;
        log_i("frm:"+str(i));
        auto p = Frm::create();
        if(!p->load(cfg_, sPath, i))
            break;
        
        //---- call
        onImg(*p);

        
        sys::sleep(1.0/lc_.fps);
    }
    return true;
}
//----
Sp<Points::Vis> Recon3d::get_frm_pnt_vis()
{
    auto& p = data_.p_pvis_frm; 
    if(p!=nullptr) return p;
    p = Points::Vis::create(cfg_.visc.pntvc);
    return p;
}
//----
void Recon3d::show(const Frm& f)
{
    auto& fd = f.data();

    //---- show undistorted imgs
    
    auto& ud_imgs = fd.ud_imgs;
    auto pL = ud_imgs[0];
    auto pR = ud_imgs[1];
    pL->show("Left undistorted");
    pR->show("Right undistorted");
    
    // show color img
    int i_c = cfg_.frms.color_img;
    if(i_c>1)
    {
        assert(i_c<ud_imgs.size());
        auto pC = ud_imgs[i_c];
       
        pC->scale(0.2);
        pC->show("Color undistorted");
    }
    //--- disparity
    auto p_imd = f.depth.p_im_disp;
    if(p_imd!=nullptr)
    {
        cv::Mat imd = img2cv(*p_imd);
        imd = imd * 0.01;
        cv::imshow("disparity", imd);
    }
    //--- local points
    if(1)
    {
        auto pv = get_frm_pnt_vis();
        assert(pv!=nullptr);
        auto& vis = *pv;
        vis.clear();
        vis.add(f.pnts, "frm");
        // dbg
        if(0)
        {
            vec3 sz; sz << 1,1,1;
            Pose T;
            vis.addCube("dbg_box", T, sz);
        }
        vis.spin();
    }
    //--- cv show spin
    cv_waitkey(10);
    
}