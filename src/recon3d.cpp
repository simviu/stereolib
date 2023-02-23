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
    assert(camcs.cams.size()>imgs.size());
  //  assert(camcs.cams.size()==ccvd.remapds.size());
    auto& cvd = *camcs.get_cvd();

    int i=0;
    for(auto p : imgs)
    {
        auto pu = cvd.remap(*p, i++);  
        data_.ud_imgs.push_back(pu);  
    }
    return true;
}

//----
bool Recon3d::Frm::genPnts(const Cfg& cfg)
{
    if(cfg.frms.depth_img>=0)
        return genPnts_byDepth(cfg);
    else if(cfg.frms.dispar_img)
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

    //---- calc disparity
    ok &= depth.calc_dispar(cfg.disp, *imgs[0], *imgs[1]);
    if(!ok) return false;
    auto p_imd = depth.p_im_disp;
    assert(p_imd);
    cv::Mat imd = img2cv(*p_imd);

    //---- calc depth
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

//-----
bool Recon3d::Frm::recon(const Cfg& cfg)
{
    bool ok = true;
    ok &= genPnts(cfg);
    return true;

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
    

    while(ok)
    {
        assert(data_.p_pvis_frm!=nullptr);
        data_.p_pvis_frm->spin();
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
        assert(i_c>=ud_imgs.size());
        auto pC = ud_imgs[i_c];
        pC->show("Color undistorted");
    }
    //--- local points
    assert(data_.p_pvis_frm!=nullptr);
    auto& vis = *data_.p_pvis_frm;
    vis.clear();
    vis.add(f.pnts, "frm");
    vis.spin();

    
}