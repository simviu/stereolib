/*
   Author: Sherman Chen
   Create Time: 2023-03-03
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "stereolib/stereolib.h"

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
protected:
};
