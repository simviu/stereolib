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
        virtual bool calc()override;
        virtual void show()const override;
        
    protected:

        bool rectify(const CamsCfg& camcs);
        bool calc_byDepth();
        
        void disp_to_depth();
        bool calc_LRC();
        bool depth_to_pnts_LRC();
        bool chk_get_depth();

        Px alignPnt(const vec3& v , const CamCfg& camc,  const Pose& T)const;
        bool chkConvDepthFmt(const cv::Mat& imdi, cv::Mat& imd)const;
    };
}

//----------
// FrmImp
//----------
// Factory
Sp<DepthGen::Frm> DepthGen::Frm::create(int i, const Cfg& cfg)
{
    return mkSp<FrmImp>(i, cfg);
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
        Frm::data_.ud_imgs.push_back(pu);  
    }
    return true;
}


//----
bool FrmImp::calc()
{
    bool ok = true;
    if(cfg.imgs.idxs.depth>=0)
        ok &= depth_to_pnts_LRC();
    else // Full pipeline L/R stereo from scratch
        ok &= calc_LRC();
    
    //--- save frm pcd
    string swdir = cfg.s_wdir + lc_.s_pcds;
    if(!sys::mkdir(swdir)) ok &= false;
    if(ok && cfg.b_save_pcd)
        ok &= pnts.save(swdir + to_string(idx) + ".pcd");

    return ok;
}

//----
Px FrmImp::alignPnt(const vec3& v , const CamCfg& camc,  const Pose& T)const
{
    // T , or T_CL, is transform from depth map camera (Left)
    //   to color camera.
    // transform to color camera frame.
    vec3 vc = T * v;  
    vec2 qc = camc.proj(vc);
    Px px = toPx(qc);
    return px;
}
//-----
void FrmImp::disp_to_depth()
{

    //---- get : b, fx
    auto& ccs = cfg.cams.cams;
    assert(ccs.size()>1);
    auto& cc0 = ccs[0].camc; // Left cam
    double b = ccs[1].T.t.norm(); // baseline
    CamCfg::Lense l; cc0.toLense(l);
    double fx = l.fx; // focal length

    //---- get disp map
    auto p_imp = Frm::data_.p_im_disp;
    assert(p_imp!=nullptr);
    auto imp = img2cv(*p_imp);

    //----
    int w = imp.cols;
    int h = imp.rows;
    cv::Mat imd(h,w, CV_32F, 0.0);    
    for(unsigned int y = 0; y < h; y++)
    {
        float* pp = (float*)imp.ptr<CV_32F>(y);       
        float* pd = (float*)imd.ptr<CV_32F>(y);       
        for(unsigned int x = 0; x < w; x++)
        {
            double d = pp[x]; // disparity
            if(d <=0)continue;
            if(std::isnan(d)||std::isinf(d))
                continue;
            
            float z = b * fx / d;
            pd[x] = z;
        }
    }
    //----
    data_.p_im_depth = mkSp<ImgCv>(imd);
}
//---
bool FrmImp::chkConvDepthFmt(const cv::Mat& imdi, cv::Mat& imd)const
{
    int tp = imdi.type();
    if(tp==CV_32F) imd = imdi;
    if(tp!=CV_16U)
    {
        log_e("chkConvDepthFmt() unsupport type:"+to_string(tp));
        return false;
    }
    //----
    Sz sz(imdi.cols, imdi.rows);
    imd = cv::Mat(sz.h, sz.w, CV_32F);
    for(int y=0;y<sz.h; y++)
    {
        float* pdi = (float*)imdi.ptr<CV_16U>(y);
        float* pd = (float*)imd.ptr<CV_32F>(y);
        for(int x=0;x<sz.w;x++)
        {
            uint16_t zi = pdi[x];
            float z = (float)zi * 0.001; // was in mm.
            pd[x] = z;

        }
    }    
    return true;
}

//----
bool FrmImp::chk_get_depth()
{
    if(data_.p_im_depth) return true;

    //---- get depth image
    int i_d = cfg.imgs.idxs.depth;
    if(i_d<0)
    {
        log_e("depth img not loadded");
        return false;
    }
    auto p_imd = imgs[i_d];
    Sz sz = p_imd->size();
    auto imd0 = img2cv(*p_imd);
    //--- chk convert
    cv::Mat imd;
    if(!chkConvDepthFmt(imd0, imd))return false;
    data_.p_im_depth = mkSp<ImgCv>(imd);


    //---- chk confidence map
    int i_dc = cfg.imgs.idxs.depthConf;
    if(i_dc<0) return true;
    auto p_imdc = imgs[i_dc];
    data_.p_im_dispConf = p_imdc;    
    return true;
}

//----
bool FrmImp::calc_byDepth()
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
bool FrmImp::calc_LRC()
{
    // img 0/1 are always L/R
    assert(imgs.size()>1);
    bool ok = true;

    //--- undistortion map(rectify)
    ok &= rectify(cfg.cams);

    //---- calc disparity
    auto& uds = Frm::data_.ud_imgs;
    assert(uds.size()>1);
    if(!calc_dispar(cfg.disp, *uds[0], *uds[1]))
        return false;

    //--- disp to depth map
    disp_to_depth();

    //---
    depth_to_pnts_LRC();
    
    log_d("gen_pnts: "+pnts.info());
    
    return true;
}




//---------------
// depth to pnts
//---------------
bool FrmImp::depth_to_pnts_LRC()
{

    //----
    auto& ccs = cfg.cams.cams;
    assert(ccs.size()>1);
    auto& cc0 = ccs[0].camc; // Left cam
    auto& T0 = ccs[0].T; // Left cam transform body to cam
    auto T0i = T0.inv();

    //---- check get depth img conf
    if(!chk_get_depth())
        return false;
    assert(data_.p_im_depth);
    auto imd = img2cv(*data_.p_im_depth);
    

    //---- get confidence map
    cv::Mat imdc; 
    auto p_imdc = Frm::data_.p_im_dispConf;
    if(p_imdc)
        imdc = img2cv(*p_imdc);
    int tp = imd.type();
    int tp2 = imdc.type();    
    //---- get color img
    int ic = cfg.imgs.idxs.color;
    assert(ic < imgs.size()); 
    Sp<Img> p_imc = nullptr; // color img may not have
    {
        auto& ud_imgs = Frm::data_.ud_imgs;
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
    //-----
    for(unsigned int y = 0; y < imd.rows; y++)
    {

        //float* prow = (float*)imd.ptr<CV_32F>(y); // disparity image
        float* pd = (float*)imd.ptr<CV_32F>(y); // depth map
        float* prowc = p_imdc ?   // confidence map
            (float*)imdc.ptr<CV_32F>(y) : nullptr;

        //----
        for(unsigned int x = 0; x < imd.cols; x++)
        {
            double z = pd[x]; // disparity
            if(z <=0)continue;
            if(std::isnan(z)||std::isinf(z))
                continue;
            //---- check confidence
            if(prowc && prowc[x] < cfg.depth.TH_confidence)
                continue; // skip this point.

            //----
            Points::Pnt p;
            //--- depth z from disparity
            //double z = b * fx / d;
            vec2 q; q << x, y;
            vec3 v = cc0.proj(q, z);

            //--- tranform to body frm
            
            vec3 vb = T0i *v;
            
            //----
            p.p = vb;
            p.c = {255,255,255,255};

            //--- get color, with alignment
            if(p_imc!=nullptr)
            {
                assert(ic < ccs.size());
                auto& c1 = ccs[ic];
                auto& camc = c1.camc;

                Px px_c = alignPnt(vb, camc, c1.T);

                //---- new px
                auto szc = p_imc->size();
                if(!szc.isIn(px_c))continue;
                Color c; imc.get(px_c, c);
                p.c = c;
            }
            //---
            pnts.add(p);
        }

    }
    return true;
}

//----
void FrmImp::show()const
{
    //--- disparity
    auto p_imd = Frm::data_.p_im_disp;
    if(p_imd!=nullptr)
    {
        cv::Mat imd = img2cv(*p_imd);
        imd = imd * 0.01;
        cv::imshow("disparity", imd);
    }
    //--- dispar confidence
    auto p_imdc = Frm::data_.p_im_dispConf;
    if(p_imdc!=nullptr)
    {
        auto imdc = img2cv(*p_imdc);
        imdc.convertTo(imdc, CV_8UC1);
        cv::imshow("disparity confidence", imdc);
    }
}
 