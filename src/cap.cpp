#include "stereolib/stereolibCv.h"

using namespace stereo;

namespace{
    struct LCfg{
    }; LCfg lc_;
}
//---- factory
Sp<StereoCap> StereoCap::create(const string& sDev)
{
    if(sDev=="dualCam")
        return mkSp<StereoCap_dcam>();

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
bool StereoCap_dcam::init()
{
    assert(cfg_.dev_ids.size()>1);
    for(int i=0;i<2;i++)
    {
        int id = cfg_.dev_ids[i];
        auto p = mkSp<cv::VideoCapture>(i);
        if(!p->isOpened())
        {
            log_e("StereoCap dual cam failed to open device:"+to_string(id));
            return false;
        }
        log_i("StereoCap dual cam open device:"+to_string(id));
        caps_.push_back(p);
    }
    return true;
}
//-----
bool StereoCap_dcam::read(CapFrms& frms)
{
    frms.imgs.clear();
    for(auto p : caps_)
    {
        cv::Mat im;
        if(!p->read(im)) return false;
        auto pIm = mkSp<ImgCv>(im);
        frms.imgs.push_back(pIm);
    }
    return true;
}
