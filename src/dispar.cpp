/*
   Author: Sherman Chen
   Create Time: 2023-02-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "stereolib/stereolibCv.h"
#include "json/json.h"

using namespace stereo;
//----
namespace{
    //---- load SGBM json cfg
    bool decode(const Json::Value& j,
                SGBM& c)
    {
        c.minDisparity = j["minDisparity"].asInt();
        c.numDisparities = j["numDisparities"].asInt();
        c.blockSize = j["blockSize"].asInt();
        c.P1 = j["P1"].asInt();
        c.P2 = j["P2"].asInt();
        c.disp12MaxDiff = j["disp12MaxDiff"].asInt();
        c.preFilterCap = j["preFilterCap"].asInt();
        c.uniquenessRatio = j["uniquenessRatio"].asInt();
        c.speckleWindowSize = j["speckleWindowSize"].asInt();
        c.speckleRange = j["speckleRange"].asInt();

        //---- filter
        //----
        auto& jw = j["wls_filter"];
        auto& wls = c.wls_filter;
        wls.en = jw["en"].asBool();
        wls.lambda = jw["lambda"].asFloat();
        wls.sigma  = jw["sigma"].asFloat();

        return true;
    
    }
    //----
    bool decode(const Json::Value& j, 
                DisparityCfg& c)
    {
        bool ok = true;
        ok &= decode(j["sgbm"], c.sgbm);
        
        //----
        c.vis_mul = j["vis_mul"].asFloat();

        return ok;
    } 
}

//-----------
bool DisparityCfg::load(const string& sf)
{

    log_i("  Load Disparity cfg :'"+sf+"'...");
    ifstream ifs(sf);
    bool ok = true;
    if(!ifs)
    {
        log_ef(sf);
        return false;
    }
    //----
    try{
        Json::Reader rdr;
        Json::Value jd;
        rdr.parse(ifs, jd);
        auto& jp = jd["disparity"];
        ok = decode(jp, *this);
        
    }
    catch(exception& e)
    {
        log_e("exception caught:"+string(e.what()));
        return false;
    }
    if(!ok) log_e(" DisparityCfg::load() json failed");

    return ok;
}


//----------------
Sp<Img> DepthGen::Frm::calc_dispar(const DisparityCfg& cfg,
                        const Img& im1, 
                        const Img& im2)const
{
    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);

   
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
    cv::Mat ims, im_dispR;
    sgbm.compute(imL, imR, ims);
//    left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
    p_matcherR->compute(imR, imL, im_dispR);

    //---
    cv::Mat imd;
    float scl = 1.0/16.0;
    //float scl = 1.0;

    int tp = ims.type();
    ims.convertTo(imd, CV_32F, scl);
    int tp1 = imd.type();

    //--- filter
    auto& wlsc = cs.wls_filter;
    if(wlsc.en)
    {
    //    wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
        auto p_fltr = cv::ximgproc::createDisparityWLSFilter(p_sgbm);
        p_fltr->setLambda(wlsc.lambda);
        p_fltr->setSigmaColor(wlsc.sigma);
    // ref    wls_filter->filter(left_disp, left, filtered_disp, right_disp);
        cv::Mat imdf;
        p_fltr->filter(imd, imL, imdf, im_dispR);
        cv::Mat im_conf = p_fltr->getConfidenceMap();    
        imd = imdf;
    }
    auto p = mkSp<ocv::ImgCv>(imd);
    return p;
}