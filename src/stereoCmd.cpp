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
    
    assert(p_vo_mng_!=nullptr);
    Cmd::add("vo", p_vo_mng_);
  
}
//----
bool StereoCmd::init(CStrs& args)
{
    
    log_i("StereoCmd init...");
    return true;
}
