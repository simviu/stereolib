/*
   Author: Sherman Chen
   Create Time: 2023-05-08
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"


using namespace stereo;

namespace{
    struct LCfg{
        string s_pcds = "pcds/";
    }; LCfg lc_;

  //---- impl DepthGen::Frm
    class FrmImp : public DepthGen::Frm
    {
    public:
        using Frm::Frm;
        virtual bool calc(const DepthGen::Cfg& cfg)override;
        virtual void show()const override;

    };
}

//----------
// FrmImp
//----------
// Factory
Sp<DepthGen::Frm> DepthGen::Frm::create(int i)
{
    return mkSp<FrmImp>(i);
}


//----
bool FrmImp::calc(const DepthGen::Cfg& cfg)
{
    bool ok = true;
    if(cfg.imgs.idxs.depth>=0)
        ok &= genPnts_byDepth(cfg);
    else if(cfg.imgs.idxs.dispar>=0)
        ok &= genPnts_byDisp(cfg);
    else // Full pipeline L/R stereo from scratch
        ok &= genPnts_byLR(cfg);
    
    //--- save frm pcd
    string swdir = cfg.s_wdir + lc_.s_pcds;
    if(!sys::mkdir(swdir)) ok &= false;
    if(ok && cfg.b_save_pcd)
        ok &= pnts.save(swdir + to_string(idx) + ".pcd");

    return ok;
}


//----
void FrmImp::show()const
{
    //--- disparity
    auto p_imd = data_.p_im_disp;
    if(p_imd!=nullptr)
    {
        cv::Mat imd = img2cv(*p_imd);
        imd = imd * 0.01;
        cv::imshow("disparity", imd);
    }
    //--- dispar confidence
    auto p_imdc = data_.p_im_dispConf;
    if(p_imdc!=nullptr)
    {
        auto imdc = img2cv(*p_imdc);
        imdc.convertTo(imdc, CV_8UC1);
        cv::imshow("disparity confidence", imdc);
    }
}
