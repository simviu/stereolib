#include "stereolib/stereolibCv.h"

using namespace stereo;

namespace{
    struct LCfg{
        //Sz dflt_sz{1280,720};
    }; LCfg lc_;
}
//---- factory
Sp<StereoCap> StereoCap::create(const string& sDev)
{
    if(sDev=="dualCam")
        return mkSp<StereoCapCv>();

    log_e("Unkown StereoCap devie:'"+sDev+"'");
    return nullptr;
}
//---- cfg
bool StereoCap::Cfg::parse(const KeyVals& kvs)
{
    //---- check devices id
    string sIds = kvs["devices"];
    if(sIds!="")
    {
        dev_ids.clear();
        auto sId_ary = tokens(sIds, ',');
        for(auto& s : sId_ary)
        {
            int id=0; 
            if(!s2d(s, id))
            {
                log_e("error id:"+to_string(id));
                return false;
            }
            dev_ids.push_back(id);

        }
    }
    

    return true;
}

//-----
bool StereoCapCv::init(const Sz& sz)
{
    int N = cfg_.dev_ids.size();
    //assert(N>1);
    for(int i=0;i<N;i++)
    {
        int id = cfg_.dev_ids[i];
        auto p = mkSp<cv::VideoCapture>(id, cv::CAP_V4L2);
        if(!p->isOpened())
        {
            log_e("StereoCap dual cam failed to open device:"+to_string(id));
            return false;
        }
        //---- set res
        //auto& sz = lc_.dflt_sz;
        if(sz.w>0)
        {
            p->set(cv::CAP_PROP_FRAME_WIDTH, sz.w);
            p->set(cv::CAP_PROP_FRAME_HEIGHT, sz.h);
        }
        //----
        log_i("StereoCap dual cam open device:"+to_string(id));
        caps_.push_back(p);

        //---
        sys::sleep(0.5);
    }
    return true;
}
//-----
bool StereoCapCv::read(CapFrms& frms)
{
    frms.imgs.clear();
    int i=0;
    for(auto p : caps_)
    {
        cv::Mat im;
        if(!p->read(im)) 
        {
            log_e("Failed to get frm from device idx=" +
                        to_string(cfg_.dev_ids[i]));
            return false;
        }
        auto pIm = mkSp<ImgCv>(im);
        auto sz = pIm->size();
        frms.imgs.push_back(pIm);
        i++;
    }
    return true;
}
