/*
   Author: Sherman Chen
   Create Time: 2023-03-03
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */
#include "stereolib/oslam3.h"
#include "stereolib/stereolibCv.h"

using namespace stereo;

namespace{
   
}

//---- factory
Sp<VO> VO::create_os3(
            const string& sf_cfg,   // Yaml cfg of ORB-SLAM
            const string& sf_voc    // VOC txt files
            )
{
    auto p = mkSp<VO_os3>();
    p->init(sf_cfg, sf_voc);
    return p;


}
//----
bool VO_os3::init(
        const string& sf_voc,    // VOC txt files
        const string& sf_cfg     // Yaml cfg of ORB-SLAM
        )
{

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    p_sys_ = mkSp<ORB_SLAM3::System>(sf_voc, sf_cfg,ORB_SLAM3::System::STEREO, true);

    return true;  
}

//----
bool VO_os3::onImg(const Img& im1, 
                   const Img& im2)
{
    if(p_sys_==nullptr) return false;
    auto& t = data_.t;
    
    cv::Mat imL = img2cv(im1);
    cv::Mat imR = img2cv(im2);
    // Pass the images to the SLAM system
    auto& fi = data_.frmIdx;
    string sfrm = "frm"+to_string(fi);
    p_sys_->TrackStereo(imL, imR, t, 
      vector<ORB_SLAM3::IMU::Point>(), sfrm);

    double dt = 1.0/cfg_.fps;
    t += dt;
    fi ++;
    return true;
}
//----
void VO_os3::onClose()
{
    if(p_sys_==nullptr)return;
    p_sys_->Shutdown();
}

//---
bool VO_os3::save(const string& sf)
{
    if(p_sys_==nullptr)
    {
        log_e("ORB-SLAM3 not init");
        return false;
    }
    //----
    p_sys_->SaveTrajectoryEuRoC(sf);
    return true;
}
