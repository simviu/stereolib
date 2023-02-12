/*
   Author: Sherman Chen
   Create Time: 2023-02-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "stereolib/stereolibCv.h"

#include "vsn/vsnLibCv.h"
//#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stereo/quasi_dense_stereo.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

using namespace stereo;
//----
namespace{
}



//----------------
bool Depth::calc(const DisparityCfg& cfg,
                 const Img& im1, 
                 const Img& im2)
{
    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);

    bool ok = true;
   
    //---------------
    // Setting Ref : 
    //   https://jayrambhia.com/blog/disparity-mpas
    //
    /*
        sgbm.SADWindowSize = 5;
        sgbm.numberOfDisparities = 192;
        sgbm.preFilterCap = 4;
        sgbm.minDisparity = -64;
        sgbm.uniquenessRatio = 1;
        sgbm.speckleWindowSize = 150;
        sgbm.speckleRange = 2;
        sgbm.disp12MaxDiff = 10;
        sgbm.fullDP = false;
        sgbm.P1 = 600;
        sgbm.P2 = 2400;
    */
    /* setting (1)
    auto p_sgbm = cv::StereoSGBM::create(
        -64, //  int minDisparity = 0, 
        16, // int numDisparities = 16, 
        3, // int blockSize = 3,
        600,  // int P1 = 0, 
        2400, // int P2 = 0, 
        10, // int disp12MaxDiff = 0,
        4, // int preFilterCap = 0, 
        1,// int uniquenessRatio = 0,
        150, // int speckleWindowSize = 0, 
        2 // int speckleRange = 0,
        // int mode = StereoSGBM::MODE_SGBM
    );
    */
    
    
    //auto p_sgbm =  cv::StereoSGBM::create(
    //    0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); // tested parameters
    auto& dispc = cfg;
    auto& cs = dispc.sgbm;
    auto p_sgbm =  cv::StereoSGBM::create(
              	cs.minDisparity ,
              	cs.numDisparities ,
              	cs.blockSize);

    auto& sgbm = *p_sgbm;
    sgbm.setP1(cs.P1);
    sgbm.setP2(cs.P2);
    sgbm.setDisp12MaxDiff(cs.disp12MaxDiff);
    sgbm.setPreFilterCap(cs.preFilterCap);
    sgbm.setUniquenessRatio(cs.uniquenessRatio);
    sgbm.setSpeckleWindowSize(cs.speckleWindowSize);
    sgbm.setSpeckleRange(cs.speckleRange);

    sgbm.setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    auto p_matcherR = cv::ximgproc::createRightMatcher(p_sgbm);
    
    //---------------
    cv::Mat imL = imc1.im_;
    cv::Mat imR = imc2.im_;
    cv::Mat im_sgbm, im_disp, im_dispR;
    sgbm.compute(imL, imR, im_sgbm);
//    left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
    p_matcherR->compute(imR, imL, im_dispR);
    float scl = 1.0; //1.0/16.0;
    im_sgbm.convertTo(im_disp, CV_32F, scl);

    //--- filter
    cv::Mat imdf;
    auto& wlsc = cs.wls_filter;

//    wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
    auto p_fltr = cv::ximgproc::createDisparityWLSFilter(p_sgbm);
    p_fltr->setLambda(wlsc.lambda);
    p_fltr->setSigmaColor(wlsc.sigma);
// ref    wls_filter->filter(left_disp, left, filtered_disp, right_disp);
    p_fltr->filter(im_disp, imL, imdf, im_dispR);

    cv::Mat im_conf = p_fltr->getConfidenceMap();    
    p_imd_ = mkSp<ocv::ImgCv>(imdf);
    return true;
}