/*
   Author: Sherman Chen
   Create Time: 2023-05-08
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"
#include "PFMReadWrite.h"

using namespace stereo;

namespace{
    struct LCfg{
    }; LCfg lc_;

  //---- impl DepthGen::Frm
    class FrmImp : public DepthGen::Frm
    {
    public:
        using Frm::Frm;
        virtual bool calc()override;
        virtual void show()const override;
        
    protected:

    //    bool rectify(const CamsCfg& camcs);
    //    bool calc_byDepth();
        
        void disp_to_depth();
        bool depth_to_pnts();
        bool calc_LRC(); // left/right/color
        bool calc_RGBD();
        bool calc_disp_color();

        bool save_out()const;

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
/*
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
*/

//----
bool FrmImp::calc()
{
    bool ok = true;
    string sm = cfg.sMode;
    if(sm == "RGBD")
        ok &= calc_RGBD();
    // Full pipeline L/R stereo from scratch
    else if(sm == "LRC")
        ok &= calc_LRC();
    else if(sm == "disp_color")
        calc_disp_color();
    else if(sm == "depth_color")
        depth_to_pnts();
    else
    {
        log_e("Unkonwn mode: "+cfg.sMode);
        return false;
    }
    //----
    ok &= save_out();
    return ok;
}

//----
bool FrmImp::calc_disp_color()
{
    //--- disp to depth map
    disp_to_depth();

    //---
    depth_to_pnts();
    
    log_d("gen_pnts: "+pnts.info());
    
    return true;
}

//----
Px FrmImp::alignPnt(const vec3& vb , const CamCfg& camc,  const Pose& T_cb)const
{
    // T_cb, transform from body to color camera
    vec3 vc = T_cb * vb;  
    vec2 qc = camc.proj(vc);
    Px px = toPx(qc);
    return px;
}
//-----
void FrmImp::disp_to_depth()
{
    auto& cams = cfg.cams;
    auto pCamL = cams.find("left");  assert(pCamL);
    auto pCamR = cams.find("right"); assert(pCamR);
    auto pCamC = cams.find("color"); assert(pCamC);

    //---- get : b, fx


    auto& cc0 = pCamL->camc; // Left cam
    auto& cc1 = pCamR->camc; // Right cam
    vec3 dt = pCamL->T.t - pCamR->T.t; // TODO: this is simplified.
    double b = dt.norm(); // baseline


    CamCfg::Lense l; cc0.toLense(l);
    double fx = l.fx; // focal length

    //---- get disp map
    auto p_imp = findImg("disp");
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
    imgs["depth"] = mkSp<ImgCv>(imd);
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
    //----5
    Sz sz(imdi.cols, imdi.rows);
    imd = cv::Mat(sz.h, sz.w, CV_32F);
    for(int y=0;y<sz.h; y++)
    {
        float* pdi = (float*)imdi.ptr<CV_16U>(y);
        float* pd = (float*)imd.ptr<CV_32F>(y);
        for(int x=0;x<sz.w;x++)
        {
            uint16_t zi = pdi[x];
            //cout << zi << ", "; // dbg
            float z = (float)zi * 0.001; // was in mm.
            pd[x] = z;

        }
    }    
    return true;
}
/*
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
*/
//----
/*
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
*/
//----
bool FrmImp::calc_LRC()
{
    assert(imgs.size()>=3);
    auto& cams = cfg.cams;
    auto pCamL = cams.find("left");  assert(pCamL);
    auto pCamR = cams.find("right"); assert(pCamR);
    auto pCamC = cams.find("color"); assert(pCamC);
    

    bool ok = true;


    //--- undistortion
    // img idx L,R,C are 0,1,2    
    auto pL = findImg("left");  assert(pL);
    auto pR = findImg("right"); assert(pR);
    auto pC = findImg("color"); assert(pC);

    //----
    if(cfg.imgs.undist_LR)
    {
        pL = pCamL->camc.undist(*pL);
        pR = pCamR->camc.undist(*pR);
    }
    if(cfg.imgs.undist_C)
        pC = pCamC->camc.undist(*pC);

    
    //---- calc disparity
    if(!calc_dispar(cfg.disp, *pL, *pR))
        return false;

    //--- disp to depth map
    disp_to_depth();

    //---
    depth_to_pnts();
    
    log_d("gen_pnts: "+pnts.info());
    
    return true;
}




//---------------
// depth to pnts
//---------------
bool FrmImp::depth_to_pnts()
{

    //----
    auto& cams = cfg.cams;
    auto pCamL = cams.find("left");  assert(pCamL);
  //  auto pCamR = cams.find("right"); assert(pCamR);
    auto pCamC = cams.find("color"); assert(pCamC);

    auto& camcL = pCamL->camc; // Left cam
    auto& camcC = pCamC->camc; // Color cam
    auto& T_Lb = pCamL->T; // T_Lb, Left cam transform body to cam
    auto T_bL = T_Lb.inv();
    auto& T_cb = pCamC->T; // color camera transform

    auto pC = findImg("color");
    auto& imc = *pC;

    //---- check get depth img conf
    auto p_imd = findImg("depth");
    assert(p_imd);
    auto imd = img2cv(*p_imd);    

    //---- get confidence map
    cv::Mat imdc; 
    auto p_imdc = findImg("dispConf");
    if(p_imdc)
        imdc = img2cv(*p_imdc);
    int tp = imd.type();
    int tp2 = imdc.type();    

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
            if(std::isnan(z)||std::isinf(z))
                continue;
            if(z <0)continue;
            auto& rng = cfg.depth.range;
            if(!rng.isIn(z))continue;
            //---- check confidence
            if(prowc && prowc[x] < cfg.depth.TH_confidence)
                continue; // skip this point.

            //----
            Points::Pnt p;
            //--- depth z from disparity
            //double z = b * fx / d;
            vec2 q; q << x, y;
            vec3 v = camcL.proj(q, z);

            //--- tranform to body frm
            
            vec3 vb = T_bL *v;
            
            //----
            p.p = vb;
            p.c = {255,255,255,255};

            //--- get color, with alignment
            Px px_c = alignPnt(vb, camcC, T_cb);

            //---- new px
            auto szc = imc.size();
            if(!szc.isIn(px_c))continue;
            Color c; imc.get(px_c, c);
            p.c = c;

            //---
            pnts.add(p);
        }

    }
    return true;
}

//---------------
// calc_RGBD
//---------------
// RGBD image aligned with depth
bool FrmImp::calc_RGBD()
{

    //----
    auto pCamC = cfg.cams.find("color");
    assert(pCamC);
    auto& camcC = pCamC->camc; 
    

    //---- check get depth img  and conf
    auto p_imdi = findImg("depth"); 
    assert(p_imdi);
    
    auto imdi = img2cv(*p_imdi);
    cv::Mat imd;
    if(!chkConvDepthFmt(imdi, imd))
        return false;
    int tp = imd.type();

    //---- get confidence map
    Sp<Img> p_imdc = nullptr;
    cv::Mat imdc;
    /*
    auto p_imdc = Frm::data_.p_im_dispConf;
    int tp2 = -1;
    if(p_imdc){
        imdc = img2cv(*p_imdc);
        int tp2 = imdc.type();   
    } 
    */

    //---- get color img at idx 0
    auto p_imc = imgs[0];

    //----
    pnts.clear();
    int k=0;
    //-----
    for(unsigned int y = 0; y < imd.rows; y++)
    {
        float* pd = (float*)imd.ptr<CV_32F>(y); // depth map
        float* pdc = p_imdc ?   // confidence map
            (float*)imdc.ptr<CV_32F>(y) : nullptr;

        //----
        for(unsigned int x = 0; x < imd.cols; x++)
        {
            double z = pd[x]; // disparity
            if(z <=0)continue;
            if(std::isnan(z)||std::isinf(z))
                continue;
            //---- check confidence
            if(pdc && pdc[x] < cfg.depth.TH_confidence)
                continue; // skip this point.

            //----
            Points::Pnt p;
            //--- depth z from disparity
            //double z = b * fx / d;
            vec2 q; q << x, y;
            vec3 v = camcC.proj(q, z);
            p.p = v;
            p.c = {255,255,255,255};

            //--- get color, with alignment
            if(p_imc!=nullptr)
            {
                Px px(x,y);
                auto szc = p_imc->size();
                if(!szc.isIn(px))continue;
                Color c; p_imc->get(px, c);
                p.c = c;
            }
            //---
            pnts.add(p);
        }

    }
    return true;
}

//----
bool FrmImp::save_out()const
{
    bool ok = true;
    //--- save frm pcd
    for(auto& s : cfg.ss_save)
    {
        string swdir = cfg.s_wdir + s +"/";
        if(!sys::mkdir(swdir)) 
            return false;
        //-----
        if(s=="pcd")
            ok &= pnts.save(swdir + to_string(idx) + ".pcd");
        
        //---- save disparity
        else if(s=="disp")
        {
            string sf = swdir + to_string(idx) + ".pfm";
            //----
            auto pd = findImg("disp");
            assert(pd!= nullptr);
            auto imd = img2cv(*pd);
            ok &= savePFM(imd, sf);

        }
    
        //---- save disparity
        else if(s=="disp_vis")
        {
            string sf = swdir + to_string(idx) + ".png";
            //----
            auto pd = findImg("disp");
            assert(pd!= nullptr);
            ok &= pd->save(sf);

        }
    }
    return ok;
}
//----
void FrmImp::show()const
{
    //----
    for(auto& s : {"left, right, color"})
    {
        auto p = findImg("s");
        if(p) p->show(s);
    }
    
    
    //--- disparity
    auto p_imd = findImg("disp");
    if(p_imd!=nullptr)
    {
        cv::Mat imd = img2cv(*p_imd);
        imd = imd * 0.01;
        cv::imshow("disparity", imd);
    }
    //--- dispar confidence
    auto p_imdc = findImg("dispConf");
    if(p_imdc!=nullptr)
    {
        auto imdc = img2cv(*p_imdc);
        imdc.convertTo(imdc, CV_8UC1);
        cv::imshow("disparity confidence", imdc);
    }
}
 