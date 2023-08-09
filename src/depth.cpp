/*
   Author: Sherman Chen
   Create Time: 2023-01-20
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"
#include "json/json.h"

using namespace stereo;

namespace{
    struct LCfg{
        float fps = 30; 
        float pnt_sz = 15;
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

//------------------------------
bool DepthGen::Cfg::load(const string& sf)
{

    log_i("Load DepthGen cfg :'"+sf+"'");
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
        //----
        sMode = jd["mode"].asString();
        //---
        {
            imgs.sDirs.clear();
            auto jimgs = jd["imgs"];
            for(auto& j : jimgs["dirs"])
                imgs.sDirs.push_back(j.asString());

            imgs.undist_LR = jimgs["undist_LR"].asBool();
            imgs.undist_C  = jimgs["undist_C"].asBool();
            
        }
        //---
        //b_save_pcd = jd["save_pcd"].asBool();
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
            depth.TH_confidence = jt.get("TH_confidence","").asDouble();
        }
        //---- others
        s_wdir = jd["wdir"].asString();
        if(s_wdir!="")
            ok &= sys::mkdir(s_wdir);
        else{
            log_e("missing 'wdir'");
            ok = false;
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
    //visc.pntvc.axisL = depth.range.d1;
    visc.pntvc.axisL = 1; // TODO: cfg
    //----
    log_i("DepthGen cfg loaded '"+sf +"'");
    return true;
}
//-----
bool DepthGen::Cfg::set(const KeyVals& kvs)
{
    string sw = kvs.get("-save");
    auto sws = tokens(sw, '|');
    for(auto& s : sws) 
        ss_save.push_back(s);

    //----
    sMode = kvs["mode"];
    if(sMode=="rgbd") 
        imgs.sDirs = {"color", "depth"};
    else if(sMode=="LRC") 
        imgs.sDirs= {"left", "right", "color"};
    else if(sMode=="disp_color") 
        imgs.sDirs = {"disp", "color"};
    else if(sMode=="depth_color") 
        imgs.sDirs = {"depth", "color"};
    else
    {
        log_e("Unkown mode : '"+sMode+"'");
        return false;
    }

    
    //----
    /*
    string sds = kvs.get("sdirs");
    if(sds!="")
        imgs.sDirs = tokens(sds, '|');
        */


    //----
    return true;
}

//----------
// DepthGen
//----------
//----
void DepthGen::init_cmds()
{
    sHelp_ = "(Depth generation)";
    string sOpts = " mode=[rgbd|disp_color|LRC] -save=[pcd|disp]";


    Cmd::add("init", mkSp<Cmd>("cfg=<CFG_FILE>",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        return cfg_.load(lookup(kv, "cfg")); 
    }));

    Cmd::add("video", mkSp<Cmd>("file=<FILE>" + sOpts,
    [&](CStrs& args)->bool{  return run_video(args);  }));

    Cmd::add("frms", mkSp<Cmd>("dir=<DIR> "+sOpts,
    [&](CStrs& args)->bool{  return run_frms(args);  }));

    Cmd::add("frm", mkSp<Cmd>("dir=<DIR> i=<IDX> "+sOpts,
    [&](CStrs& args)->bool{   return run_frm(args);  }));
}



//---
bool DepthGen::onImg(Frm& f)
{
    
    //---- recon
    bool ok = true;
    ok &= f.calc();
    
    //--- show
    show(f);
    return true;

}

//----
bool DepthGen::run_frm(CStrs& args)
{
    KeyVals kvs(args);
    string sPath =kvs["dir"]; 
    int i=-1; 
    if(!kvs.get("i", i))return false;
    if(!cfg_.set(kvs))return false;

    log_i("frm:"+str(i));
    auto p = Frm::create(i, cfg_);
    if(!p->load(cfg_, sPath, i))
        return false;
    
    

    //---- call
    bool ok = onImg(*p);
    if(!ok)
    {
        log_e("DepthGen::run_frm() failed");
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
bool DepthGen::run_video(CStrs& args)
{

    KeyVals kvs(args);
    string sf =kvs["file"]; 
    if(!cfg_.set(kvs))return false;
  //----
    int i=0;
    auto pv = Video::open(sf);
    if(pv==nullptr)return false;

    while(1)
    {
        i++;
        log_i("frm:"+str(i));
        auto p = Frm::create(i, cfg_);
        if(!p->load(*pv))
            break;
        
        //---- call
        onImg(*p);

        sys::sleep(1.0/lc_.fps);
    }
    return true;
}

//----
bool DepthGen::run_frms(CStrs& args)
{
    KeyVals kvs(args);
    string sPath =kvs["dir"]; 
    if(!cfg_.set(kvs))return false;
    //----
    int i=0;
    while(1)
    {
        i++;
        log_i("frm:"+str(i));
        auto p = Frm::create(i, cfg_);
        if(!p->load(cfg_, sPath, i))
            break;
        
        //---- call
        onImg(*p);
        
        sys::sleep(1.0/lc_.fps);
    }
    return true;
}
//----
Sp<Points::Vis> DepthGen::get_frm_pnt_vis()
{
    auto& p = data_.p_pvis_frm; 
    if(p!=nullptr) return p;
    p = Points::Vis::create(cfg_.visc.pntvc);
    return p;
}
//----
void DepthGen::show(const Frm& f)
{
    auto& visc = cfg_.visc;
    //---show frm
    f.show();
    
    //--- local points
    if(1)
    {
        auto pv = get_frm_pnt_vis();
        assert(pv!=nullptr);
        auto& vis = *pv;
        vis.clear();
        vis.add(f.pnts, "frm", lc_.pnt_sz);
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