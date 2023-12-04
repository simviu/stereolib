#include "stereolib/stereolibCv.h"


using namespace stereo;

namespace{
    struct LCfg{
        float scl_vs = 0.5; // scale fo vis
    }; LCfg lc_;
    
}



//-------
void StereoCmd::init_cmds()
{
    sHelp_ = "(Stereo Vision commands)";

    //----
    add("init", mkSp<Cmd>("file=[NAME]",
    [&](CStrs& args)->bool{ 
       return init(args);
    }));
    //----
    string sH = "fps=<FPS> devices=<ID0,ID1> res=<W,H>";
    sH += " vis=false save=<DIR>";
    add("cap", mkSp<Cmd>(sH,
    [&](CStrs& args)->bool{ 
       return capFrms(args);
    }));
    //-----
    sH = "dir=<IMG_DIR> [wait_key=1] (stereo calibration)\n";
    sH += "    (wait_key is frm delay, set 0 for key pause)";
    add("calib", mkSp<Cmd>(sH,

    [&](CStrs& args)->bool{ 
       return run_stereo_calib(args);
    }));

    //---- modules    
    Cmd::add("vo",      mkSp<VO_mng>());
    Cmd::add("depth",   mkSp<DepthGen>());    
    Cmd::add("recon",   mkSp<ReconScn>());

  
}
//----
bool StereoCmd::init(CStrs& args)
{
    
    log_i("StereoCmd init...");
    return true;
}

//---- TODO: merge with vsnLib/cmdImg
bool StereoCmd::capFrms(CStrs& args)
{
    KeyVals kvs(args);
    log_i("capture video frms...");
    //-----
    auto pCap = StereoCap::create("dualCam");
    if(pCap==nullptr) return false;
    if(!pCap->cfg_.parse(kvs)) return false;
    //----
    string sWd = kvs["save"];
    auto& sNames = pCap->cfg_.sNames;
    if(sWd!="")
    {
        sys::mkdir(sWd);
        for(auto& s : sNames)
            sys::mkdir("./"+ sWd + "/" + s);
    }
    //---- chk res
    Sz sz(-1,-1);
    auto tks = tokens(kvs["res"], ',');
    bool ok = (tks.size()>1) &&  
            s2d(tks[0], sz.w) &&
            s2d(tks[1], sz.h) ;
    if(!ok){ log_e("wrong res"); return false; }
    
    //--- init
    if(!pCap->init(sz)) return false;
    //-----
    float fps=10;
    ok = s2d(kvs["fps"], fps);
    if((!ok)||(fps<=0))
    {
        log_e("Incorrect fps:"+kvs["kvs"]);
        return false;
    }
    //----
    float dt = 1.0/fps;
    int fi=0; // frame idx
    while(1)
    {
        fi++;
        StereoCap::CapFrms frms;
        if(!pCap->read(frms))
        {
            log_e("StereoCap dual cam fail to get frms");
            return false;
        }

        int N= frms.imgs.size();
        
        //---- save
        if(sWd!="")
            for(int i=0;i<N;i++)
            {
                auto pIm = frms.imgs[i];
                string sfw = "./" + sWd +  "/" +
                            sNames[i] +"/" + to_string(fi) + ".png";
                if(!pIm->save(sfw)) return false;
            }
        //---- show 
        if(kvs["vis"]=="true")
        {

            //if(N > 2) N=2;
            for(int i=0;i<N;i++)
            {
                auto pIm = frms.imgs[i];
                pIm->scale(lc_.scl_vs);
                pIm->show(sNames[i]);
            }
        }
    }

    return true;
}

//-----
bool StereoCmd::run_stereo_calib(CStrs& args)
{
    KeyVals kvs(args);
    string spath = kvs["dir"];
    StereoCalib calib;
    int wk = -1;
    if(s2d(kvs["wait_key"], wk))
        calib.cfg_.wait_key = wk;
    return calib.calb_imgs(spath);
}