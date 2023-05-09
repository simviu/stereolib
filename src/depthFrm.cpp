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
    protected:
        bool rectify(const CamsCfg& camcs);
        bool calc_byDepth(const DepthGen::Cfg& cfg);
        bool calc_byDisp(const DepthGen::Cfg& cfg);
        bool calc_byLR(const DepthGen::Cfg& cfg);
        void disp_to_pnts(const DepthGen::Cfg& cfg);
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
bool FrmImp::rectify(const CamsCfg& camcs)
{
  //  auto& ccvd = cast_imp(*camcs.get_cvd());
    int N = camcs.cams.size();
    assert(N<=imgs.size());
  //  assert(camcs.cams.size()>imgs.size());
  //  assert(camcs.cams.size()==ccvd.remapds.size());
    auto& cvd = *camcs.get_cvd();

    for(int i=0;i<N;i++)
    {
        //auto pu = cvd.remap(*imgs[i], i);  
        auto& cc = camcs.cams[i].camc; 
        auto pu = cc.undist(*imgs[i]);
        data_.ud_imgs.push_back(pu);  
    }
    return true;
}


//----
bool FrmImp::calc(const DepthGen::Cfg& cfg)
{
    bool ok = true;
    if(cfg.imgs.idxs.depth>=0)
        ok &= calc_byDepth(cfg);
    else if(cfg.imgs.idxs.dispar>=0)
        ok &= calc_byDisp(cfg);
    else // Full pipeline L/R stereo from scratch
        ok &= calc_byLR(cfg);
    
    //--- save frm pcd
    string swdir = cfg.s_wdir + lc_.s_pcds;
    if(!sys::mkdir(swdir)) ok &= false;
    if(ok && cfg.b_save_pcd)
        ok &= pnts.save(swdir + to_string(idx) + ".pcd");

    return ok;
}


//----
bool FrmImp::calc_byDepth(const DepthGen::Cfg& cfg)
{
    int i_d = cfg.imgs.idxs.depth;
    assert(i_d<imgs.size());
    auto pd = imgs[i_d];
    assert(pd!=nullptr);
    

    // assume depth aligned with RGB
    int i_c = cfg.imgs.idxs.color;
    assert(i_c<imgs.size());
    auto pc = imgs[i_c];

    auto& cc_c = cfg.cams.cams[i_c].camc;
    Sz sz = cc_c.sz;
    //pd->scale(sz);
    ImgCv imcc(*pc);
    ImgCv imcd(*pd);
    cv::Mat imc = imcc.im_;
    cv::Mat imd = imcd.im_;
    cv::resize(imd, imd, cv::Size(sz.w, sz.h), cv::INTER_NEAREST);
    int tpd = imd.type();
    cv::Mat K; cv::eigen2cv(cc_c.K, K);
    for(int i=0;i<imc.rows;i++)
        for(int j=0;j<imc.cols;j++)
        {
            auto& dx = imd.ptr<uint16_t>(i)[j];
        //    double d = fp16_to_float(dx);
            double d = dx;
            double z = d;//*0.001; // was mm
            auto c = imc.ptr<const BGR>(i)[j];
            if(dx!=0)
            { int dbg=0;}
            Points::Pnt p;
            p.c = {c.r,c.g,c.b,255};
            vec2 px; px << j,i;
            p.p = cc_c.proj(px, z);
            pnts.add(p);
        }
    return true;
}
//----
bool FrmImp::calc_byLR(const DepthGen::Cfg& cfg)
{
    // img 0/1 are always L/R
    assert(imgs.size()>1);
    bool ok = true;

    //--- undistortion map(rectify)
    ok &= rectify(cfg.cams);

    //---- calc disparity
    auto& uds = data_.ud_imgs;
    assert(uds.size()>1);
    if(!calc_dispar(cfg.disp, *uds[0], *uds[1]))
        return false;

    //---
    disp_to_pnts(cfg);
    
    log_d("gen_pnts: "+pnts.info());
    
    return true;
}
//----
bool FrmImp::calc_byDisp(const DepthGen::Cfg& cfg)
{

    //--- if depth image is disparity
    /*
    cv::Mat Q; // Q from stereoRectify
    cv::eigen2cv(cfg.cams.rectify.Q, Q);
    Q.convertTo(Q, CV_64FC1);
    int tpQ = Q.type();
    cv::Mat im3d;
    cv::Mat imd; imcd.im_.convertTo(imd, CV_8UC1);

    int tpd = imd.type();
    cv::reprojectImageTo3D(imd, im3d, Q);
    int tp3 = im3d.type();
    */
    //----
    log_e("not yet");
    return false;
}
//----
/*
bool DepthGen::Frm::renderPnts(const Cfg& cfg)
{
    int ic = cfg.frms.color_img; 
    if(ic<0) return false; // no color
    assert(ic < imgs.size());
    auto p_imc = imgs[ic];
    for(auto& p : pnts.getData())
    {

    }
    return true;
}
*/


//---------------
// calc_disp_to_pnts
//---------------
// This disparity map calced by 
//   StereoSGBM, divided by 16.
// The depth generation by undistorted imgs.
// The points generated by depth, re-project by
// Color camera's transform and intrinsic.
// So un-aligned cameras supported.
void FrmImp::disp_to_pnts(const DepthGen::Cfg& cfg)
{
    //----
    auto& ccs = cfg.cams.cams;
    assert(ccs.size()>1);
    auto& cc0 = ccs[0].camc; // Left cam
    double b = ccs[1].T.t.norm(); // baseline
    CamCfg::Lense l; cc0.toLense(l);
    double fx = l.fx; // focal length
    //---- get disparity and color
    int ic = cfg.imgs.idxs.color;
    assert(ic < imgs.size()); 
    auto p_imd = data_.p_im_disp;
    assert(p_imd!=nullptr);
    auto imd = img2cv(*p_imd);
    //---- get confidence map
    cv::Mat imdc; auto p_imdc = data_.p_im_dispConf;
    if(p_imdc)
        imdc = img2cv(*p_imdc);
    
    //----
    Sp<Img> p_imc = nullptr; // color img may not have
    {
        auto& ud_imgs = data_.ud_imgs;
        assert(ic<ud_imgs.size());
        if(ic>=0) p_imc = ud_imgs[ic];
    }
    //auto imc = img2cv(*p_imc);
    auto& imc = *p_imc;
    //int tp1 = imc.type();// dbg
    //assert(tp1 == CV_8UC4);
    //----
    pnts.clear();
    int k=0;
    int tp = imd.type(); // dbg
    for(unsigned int y = 0; y < imd.rows; y++)
    {
        float* prow = (float*)imd.ptr<CV_32F>(y); // disparity image
        float* prowc = p_imdc ?   // confidence map
            (float*)imdc.ptr<CV_32F>(y) : nullptr;

        //----
        for(unsigned int x = 0; x < imd.cols; x++)
        {
            double d = prow[x]; // disparity
            if(d <=0)continue;
            if(std::isnan(d)||std::isinf(d))
                continue;
            //---- check confidence
            if(prowc && prowc[x] < cfg.depth.TH_confidence)
                continue; // skip this point.

            //----
            Points::Pnt p;
            //--- calc location
            double z = b * fx / d;
            vec2 q; q << x, y;
            vec3 v = cc0.proj(q, z);
            p.p = v;
            //--- get color
            p.c = {255,255,255,255};
            if(p_imc!=nullptr)
            {
                assert(ic < ccs.size());
                auto& c1 = ccs[ic];
                auto& camc = c1.camc;
                auto& T = c1.T;
                auto sz = p_imc->size();
                // transform to color camera frame.
                vec3 v1 = T * v;  
                vec2 q1 = camc.proj(v1);
                Px px = toPx(q1);

                if(!sz.isIn(px))continue;
                Color c; imc.get(px, c);
                p.c = c;
            }
            //---
            pnts.add(p);
        }

    }
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
