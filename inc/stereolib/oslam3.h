/*
   Author: Sherman Chen
   Create Time: 2023-03-03
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "stereolib/stereolib.h"
#include "System.h"

using namespace stereo;
//Stereo video odometry
class VO_os3 : public VO{
public:
    
    virtual bool onImg(const Img& im1, 
                       const Img& im2)override;
    virtual bool genDepth(const Img& im1,  
                          const Img& im2,
                          Depth& depth)override
                        { return false; }
    virtual bool save(const string& sf)override;
    virtual void onClose()override;
    bool init(
            const string& sf_voc,    // VOC txt files
            const string& sf_cfg     // Yaml cfg of ORB-SLAM
            );
    
    
protected:

    Sp<ORB_SLAM3::System> p_sys_ = nullptr;
};
