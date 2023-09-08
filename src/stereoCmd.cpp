#include "stereolib/stereolib.h"


using namespace stereo;

namespace{
    struct LCfg{
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
    add("cap", mkSp<Cmd>("fps=<FPS> devices=<ID0,ID1> vis=false (capture stereo frms to output/frms/)",
    [&](CStrs& args)->bool{ 
       return capFrms(args);
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

//----
bool StereoCmd::capFrms(CStrs& args)
{
    KeyVals kvs(args);
    log_i("capture video frms...");
    //-----
    auto pCap = StereoCap::create("dualCam");
    if(pCap==nullptr) return false;
    if(!pCap->cfg_.parse(kvs)) return false;
    //--- init
    if(!pCap->init()) return false;
    //-----
    float fps=10;
    bool ok = s2d(kvs["fps"], fps);
    if((!ok)||(fps<=0))
    {
        log_e("Incorrect fps:"+kvs["kvs"]);
        return false;
    }
    //----
    float dt = 1.0/fps;
    while(1)
    {
        StereoCap::CapFrms frms;
        if(!pCap->read(frms))
        {
            log_e("StereoCap dual cam fail to get frms");
            return false;
        }

        //---- show 
        if(kvs["vis"]=="true")
        {

            int N= frms.imgs.size();
            //if(N > 2) N=2;
            auto& sNames = pCap->cfg_.sNames;
            for(int i=0;i<N;i++)
            {
                auto pIm = frms.imgs[i];
                pIm->show(sNames[i]);
            }
        }
    }

    return true;
}
