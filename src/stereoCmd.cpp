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
