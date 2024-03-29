/*
   Author: Sherman Chen
   Create Time: 2023-02-06
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "stereolib/stereolib.h"
#include "vsn/vsnLibCv.h"
//#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stereo/quasi_dense_stereo.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

namespace stereo
{
    //Stereo video odometry
    class VOcv : public VO{
    public:
        //--- Triangulation 3d pnt of
        // Stereo match pnt
        struct MPnt{
            int mi=-1; // index to matches
            cv::Point3f Pt; // 3d triangulation pnt
            cv::Point3f Pd; // 3d pnt by depth
        };

        //--- frm data
        struct FrmCv{
            Sp<FeatureMatchCv> p_fm = nullptr;
            // 3d triangulation of matched feature points.
            // (size of mpnts same as matched feature pairs)
            vector<MPnt> mpnts; 

            //--- inliers mi set after solving 2d/3d
            //set<int> inliers;
            //--- find mpnt by feature index
            bool find(int i, bool bLeft, MPnt& mpnt)const;
            bool at(int mi, MPnt& mpnt)const;
        };
        //--- cv data
        struct Data{
            //---- previous feature match
            Sp<FrmCv> p_frm_prev = nullptr;
        };
        Data data_;
        //----
        virtual bool onImg(const Img& im1, 
                        const Img& im2)override;

        virtual bool genDepth(const Img& im1,  
                            const Img& im2,
                            Depth& depth)override;
        virtual bool save(const string& sf)override
        { log_e("not yet"); return false; }

    protected:
        bool odometry(const FrmCv& frm1,
                    const FrmCv& frm2);
        bool solve_2d3d(const FrmCv& frm1,
                        const FrmCv& frm2,
                        bool bLeft,
                        cv::Mat& r, cv::Mat& t,
                        set<int>& inliers)const;
        bool triangulate(const FeatureMatchCv& fm,
                        vector<MPnt>& mpnts)const;
        void calc_pnts(const FrmCv& frmc,
                    const set<int>& mi_ary,
                    vec3s& Ps)const;
        bool genDense(const Img& imL);
        void show();

        //----
        bool run_sgbm(const Img& im1,
                    const Img& im2,
                    Depth& depth);
        bool run_quasi(const Img& im1,
                    const Img& im2,
                    Depth& depth);

    };
    //---- impl Recon3d::Frm
    class ReconFrm : public Recon3d::Frm
    {
    public:
        virtual bool calc(const Recon3d::Cfg& cfg)override;

    };
    //---- cv data
    class CamsCfgCvd : public CamsCfg::CvData
    {
    public:
        /*
        struct RemapD{
            cv::Mat map1, map2;
        };     
        vector<RemapD> remapds; // for each camera
        */
        //Q matrix from stereo rectify
        cv::Mat Q;
        //virtual Sp<Img> remap(Img& im, int cam_id)const override;
    };
    inline auto& cast_imp(const CamsCfg::CvData& d)
    { return reinterpret_cast<const CamsCfgCvd&>(d); }

    //--------
    // stereo algorithm
    //--------
    extern void calc_disp_to_pnts_cv(
            const Recon3d::Cfg& cfg,
            cv::Mat imd, Points& pnts);
   
    extern void calc_im3d_to_pnts(
            const Recon3d::Cfg& cfg,
            cv::Mat im3d, Points& pnts);
} // stereo
