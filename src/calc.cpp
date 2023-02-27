/*
   Author: Sherman Chen
   Create Time: 2023-02-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "stereolib/stereolibCv.h"
using namespace stereo;
//----
namespace{
}


//---------------
// calc_disp_to_pnts
//---------------
void stereo::calc_disp_to_pnts(
        const Recon3d::Cfg& cfg,
        cv::Mat imd, Points& pnts)
{
    //----
    auto& ccs = cfg.cams.cams;
    assert(ccs.size()>1);
    auto& cc0 = ccs[0].camc; // Left cam
    double b = ccs[1].T.t.norm(); // baseline
    CamCfg::Lense l; cc0.toLense(l);
    double fx = l.fx; // focal length
    //----
    
    pnts.clear();
    int k=0;
    int tp = imd.type();
    for(unsigned int y = 0; y < imd.rows; y++)
    {
        float* prow = (float*)imd.ptr<CV_32F>(y);
        for(unsigned int x = 0; x < imd.cols; x++)
        {
            double d = prow[x]; // disparity
            if(d <=0)continue;
            if(std::isnan(d)||std::isinf(d))
                continue;

            double z = b * fx / d;
            vec2 q; q << x, y;
            //---
            Points::Pnt p;
            p.p = cc0.proj(q, z);
            p.c = {255,255,255,255};
            pnts.add(p);
        }

    }
}

//---------------
// calc_disp_to_pnts
//---------------
void stereo::calc_disp_to_pnts_cv(
        const Recon3d::Cfg& cfg,
        cv::Mat imd, Points& pnts)
{
    //---- calc depth
    auto& cvd = cast_imp(*cfg.cams.get_cvd());
    cv::Mat im3d;
    cv::reprojectImageTo3D(imd, im3d, cvd.Q);
    //---- dbg
    double min, max;
    cv::minMaxLoc(imd, &min, &max);
    calc_im3d_to_pnts(cfg, im3d, pnts);

}
//---------------
// im3d_to_pnts
//---------------
// im3d is points generated by cv::reprojectImageTo3D
void stereo::calc_im3d_to_pnts(
        const Recon3d::Cfg& cfg,
        cv::Mat im3d, Points& pnts)
{
    pnts.clear();
    int k=0;
    int tp = im3d.type();
    for(unsigned int i = 0; i < im3d.rows; i++)
    {
        cv::Vec3f *point = im3d.ptr<cv::Vec3f>(i);

        for(unsigned int j = 0; j < im3d.cols; j++)
        {
            Points::Pnt p;

            double x,y,z;
            x = point[j][0];
            y = point[j][1];
            z = point[j][2];
            // filter invalid
            if(!(isValid(x) && isValid(y) && isValid(z)))
            {
                k++;
                continue;
            }
            //---- filter depth range
            auto& dc = cfg.depth;
            if(!dc.range.isIn(z))
                continue;
            //----
            p.p << x,y,z;
            p.c = {255,255,255,255};
            pnts.add(p);
        }
    }
    //----
    log_d("  im3d_to_pnts() invalid n="+to_string(k));
}
