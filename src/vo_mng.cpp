/*
   Author: Sherman Chen
   Create Time: 2023-03-06
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolib.h"
#include "vsn/vsnLib.h"

using namespace stereo;
//----
namespace{

}

void VO_mng::init_cmds()
{
    //----
    add("init", mkSp<Cmd>("cfg=<CFG_FILE>",
    [&](CStrs& args)->bool{ 
       return init(args);
    }));
    //----
    add("init_os3", mkSp<Cmd>("voc_file=<VOC_FILE> cfg=<CFG_FILE>",
    [&](CStrs& args)->bool{ 
       return init_os3(args);
    }));
    //----
    add("frms", mkSp<Cmd>("pathL=<PATH_L> pathR=<PATH_R>",
    [&](CStrs& args)->bool{ 
       return run_frms(args);
    }));
}
//----
bool VO_mng::init(CStrs& args)
{
    KeyVals kvs(args);
    string sfc;
    if(!kvs.get("cfg", sfc)) return false;
    p_vo_ = VO::create();
    bool ok = p_vo_->cfg_.load(sfc);
    return ok;    
}
//----
bool VO_mng::init_os3(CStrs& args)
{
    KeyVals kvs(args);
    string sfv, sfc;
    if(!kvs.get("voc", sfv)) return false;
    if(!kvs.get("cfg", sfc)) return false;
    p_vo_ = VO::create_os3(sfv, sfc);
    return true;
    
}
//-----
bool VO_mng::run_frms(CStrs& args)
{
    return true;
}
