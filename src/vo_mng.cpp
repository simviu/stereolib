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
    struct LC{
        string sf_traj = "./traj.txt";
    }; LC lc_;
}

void VO_mng::init_cmds()
{
    //----
    string sH;
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
    add("frms", mkSp<Cmd>("dir=<PATH>",
    [&](CStrs& args)->bool{ 
       return run_frms(args);
    }));
    //----
    add("video", mkSp<Cmd>("file=<FILE>",
    [&](CStrs& args)->bool{ 
       return run_video(args);
    }));
    //----
    add("cams", mkSp<Cmd>("[resize=640,362] [-save_frms]",
    [&](CStrs& args)->bool{ 
       return run_dualCams(args);
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
    
    if(!chk_init()) return false;
    //-----
    bool ok = true;
    KeyVals kvs(args);
    string sdir; 
    if(!kvs.get("dir", sdir)) return false;
    
    int i=0;
    while(1)
    {
        i++;
        string sf = to_string(i)+".png";
        auto pL = Img::loadFile(sdir+"/L/"+sf);
        auto pR = Img::loadFile(sdir+"/R/"+sf);
        if(pL==nullptr || pR==nullptr)break;

        //---- dbg show
        pL->show("Left");
        pR->show("Right");
        //----
        p_vo_->onImg(*pL, *pR);

        //----
        if(cv_waitESC(1))break;
    }
    //--- 
    log_i("VO finished, saving traj...");
    p_vo_->onClose();
    ok &= p_vo_->save(lc_.sf_traj);
    return ok;
}

//-----
bool VO_mng::run_video(CStrs& args)
{
    if(!chk_init()) return false;
    //-----
    bool ok = true;
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
        Rect r1(Px(sz.w*0.5,sz.h*0.5), sz);
        Rect r2(Px(sz.w*1.5,sz.h*0.5), sz);
        auto pL = p->crop(r1);
        auto pR = p->crop(r2);

        if(pL==nullptr || pR==nullptr)
        {
            log_e("croping L/R failed");
            return false;
        }
        //---- dbg show
        pL->show("Left");
        pR->show("Right");
        //----
        p_vo_->onImg(*pL, *pR);

        //----
        if(cv_waitESC(1))break;
    }
    //--- 
    log_i("VO finished, saving traj...");
    p_vo_->onClose();
    ok &= p_vo_->save(lc_.sf_traj);
    return ok;
}

//-----
bool VO_mng::run_dualCams(CStrs& args)
{
    if(!chk_init()) return false;
    //-----
    KeyVals kvs(args);
    string ssz = kvs.get("resize");
    Sz sz; bool bSz = false;
    bool ok = true;
    if(ssz!="" )
        ok = sz.set(ssz);
    if(!ok){ log_e("fail to parse sz '"+ssz+"'"); }
    //----
    bool b_wf = kvs.has("-save_frms");
    if(b_wf) sys::mkdir("./frms");
    //----
    auto pv0 = Video::open(0);
    auto pv1 = Video::open(1);
    if((pv0==nullptr)||pv1==nullptr)
        return false;

    int i=0;
    while(1)
    {
        i++;

        auto pL = pv0->read();
        auto pR = pv1->read();
        if(pL==nullptr || pR==nullptr)
            break;
        //---- scale
        if(bSz)
        {
            pL->scale(sz);
            pR->scale(sz);
        }
        //---- dbg show
        pL->show("Left");
        pR->show("Right");
        //--- save frm
        if(b_wf)
        {
            string sf = to_string(i)+".png";
            ok &= pL->save("./frms/L/"+sf);
            ok &= pL->save("./frms/L/"+sf);
            if(!ok)break;
        }
        //----
        p_vo_->onImg(*pL, *pR);
        if(cv_waitESC(1))break;
    }
    //----
    log_i("VO finished, saving traj...");
    p_vo_->onClose();
    ok &= p_vo_->save(lc_.sf_traj);
    return ok;
}
