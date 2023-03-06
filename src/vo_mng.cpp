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
     //----
    add("video", mkSp<Cmd>("file=<FILE>",
    [&](CStrs& args)->bool{ 
       return run_video(args);
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
bool VO_mng::chk_init()const
{
    if(p_vo_!=nullptr) return true;
    log_e("VO not init, call 'init' or 'init_os3'");
    return false;
}

//-----
bool VO_mng::run_frms(CStrs& args)
{
    log_e("not yet");
    return false;
}

//-----
bool VO_mng::run_video(CStrs& args)
{
    if(!chk_init()) return false;
    //-----
    KeyVals kvs(args);
    string sf; 
    if(!kvs.get("file", sf)) return false;

    auto p_vid = Video::open(sf);
    if(p_vid==nullptr)return false;

    while(1)
    {
        Sp<Img> p = p_vid->read();
        if(p==nullptr)break;
        Sz sz = p->size();
        sz.w *= 0.5;
        Rect r1({sz.w*0.5,sz.h*0.5}, sz);
        Rect r2({sz.w*1.5,sz.h*0.5}, sz);
        auto pL = p->crop(r1);
        auto pR = p->crop(r2);

        if(pL==nullptr || pR==nullptr)
        {
            log_e("croping L/R failed");
            return false;
        }
        //----
        p_vo_->onImg(*pL, *pR);
    }
    return true;
}
