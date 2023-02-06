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

    assert(p_recon_!=nullptr);
    Cmd::add("recon", p_recon_);    
    //----
    /*
    add("moveto", mkSp<Cmd>("xyz=x,y,z rvec=rx,ry,rz grip=[0:1]",
    [&](CStrs& args)->bool{ return moveto(args); }));
    //----
    add("st", mkSp<Cmd>("(get status)",
    [&](CStrs& args)->bool{ 
        return getSt();
    }));
    */
  
  
}
//----
bool StereoCmd::init(CStrs& args)
{
    
    log_i("StereoCmd init...");
    return true;
}

