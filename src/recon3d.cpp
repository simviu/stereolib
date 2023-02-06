/*
   Author: Sherman Chen
   Create Time: 2023-01-20
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolib.h"
#include "json/json.h"
#include "vsn/vsnLibCv.h"

using namespace stereo;

namespace{
    struct LCfg{
        float fps = 30; 
    }; LCfg lc_;
    
}
//----
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
//----
bool Recon3d::onImg(Frm& f)
{
    // show color
    if(0)
    {
        int i = cfg_.frms.color_img;
        assert(i<f.imgs.size());
        auto p = f.imgs[i];
        if(p!=nullptr)
            p->show("Color");
    }
    //---- show undistorted img L
    {
        auto p = f.imgs[0];
        assert(p!=nullptr);
        p->show("Left");
        auto& cc = cfg_.cams.cams[0].camc;
        auto pu = cc.p_undist_;
        assert(pu!=nullptr);
        auto p1 = pu->remap(*p);
        //----
        p1->show("Left undist");
    }
    //---- recon
    bool ok = true;
    ok &= f.recon(cfg_);


    return true;

}
//----
bool Recon3d::Frm::genPnts(const Cfg& cfg)
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
    pd->scale(sz);
    ImgCv imcc(*pc);
    ImgCv imcd(*pd);
    cv::Mat imc = imcc.im_;
    cv::Mat imd = imcd.im_;
    int tpd = imd.type();
    cv::Mat K; cv::eigen2cv(cc_c.K, K);
    for(int i=0;i<imc.rows;i++)
        for(int j=0;j<imc.cols;j++)
        {
            auto& d = imd.ptr<uint16_t>(i)[j];
            double z = d*0.001; // was mm
            auto c = imc.ptr<const BGR>(i)[j];
            Points::Pnt p;
            p.c = {c.r,c.g,c.b,255};
            vec2 px; px << j,i;
            p.p = cc_c.proj(px, z);
            pnts.add(p);
        }


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

    return true;
}
//----
bool Recon3d::run_frm(const string& sPath, int i)
{

    log_i("frm:"+str(i));
    Frm f;
    if(!f.load(cfg_, sPath, i))
        return false;
    
    //---- call
    onImg(f);

    //--- show
    show(f);

    while(1)
    {
        assert(data_.p_pvis_frm!=nullptr);
        data_.p_pvis_frm->spin();
        sys::sleep(1.0/lc_.fps);    
    }
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
        Frm f;
        if(!f.load(cfg_, sPath, i))
            break;
        
        //---- call
        onImg(f);

        //--- show
        show(f);
        sys::sleep(1.0/lc_.fps);
    }
    return true;
}
//-----
bool Recon3d::Frm::recon(const Cfg& cfg)
{
    bool ok = true;
    ok &= genPnts(cfg);
    return true;

}
//----
void Recon3d::show(const Frm& f)
{
    assert(data_.p_pvis_frm!=nullptr);
    auto& vis = *data_.p_pvis_frm;
    //--- local points
    vis.clear();
    vis.add(f.pnts, "frm");
    vis.spin();

}