/*
   Author: Sherman Chen
   Create Time: 2022-05-04
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
    const struct{
        int N_th_pnp = 4;
        float pnt_sz = 3;
    }lcfg_;

}


//---- Factory
Sp<VO> VO::create()
{
    return mkSp<VOcv>();
}

//-----------
bool VOcv::FrmCv::find(int i, bool bLeft, MPnt& mpnt)const
{
    if(p_fm==nullptr) return false;
    auto& fm = *p_fm;
    auto& md = fm.data_.md;
    auto& i_mi = (bLeft)?md.i1_mi : md.i2_mi;
    auto it = i_mi.find(i);
    if(it==i_mi.end())return false;
    int mi = it->second;
    if(mi >= mpnts.size()) return false;
    mpnt = mpnts[mi];
    return true;
}
//-----------
bool VOcv::FrmCv::at(int mi, MPnt& mpnt)const
{
    if(mi<0 || mi>= mpnts.size())
        return false;
    mpnt = mpnts[mi];
    return true;
}
 
//-----------
bool VOcv::onImg(const Img& im1,  
                 const Img& im2)
{
    auto& camc = cfg_.camc;
    auto& vod = VO::data_;
    auto& fi = vod.frmIdx;
    fi++;
    //--- initial file wr
    if(fi<=1 && cfg_.run.enWr) 
        vod.wr.open();

    //---- img undistort
    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);
    imc1.undistort(camc);
    imc1.undistort(camc);

    bool ok = true;
    //---- do feature matching of L/R
    auto p_fm = mkSp<FeatureMatchCv>();
    auto& fm = *p_fm;
    fm.cfg_.bShow = cfg_.run.bShow;
    fm.cfg_.N = cfg_.feature.Nf;
    ok &= fm.onImg(im1, im2);

    //---- construct frm
    auto p_frm = mkSp<FrmCv>();
    auto& frm = *p_frm;
    frm.p_fm = p_fm;

    //---- trangulate feature points.
    ok &= triangulate(fm, frm.mpnts);

    //---- gen depth
    auto p_frmo = mkSp<VO::Frm>();
    auto& frmo = *p_frmo;
    VO::data_.p_frm = p_frmo;
    auto& depth = frmo.depth;
 //   if(cfg_.run.enDepth)
 //       ok &= genDepth(im1, im2, depth);

    //---- gen denth map
 //   if(cfg_.run.enDense)
 //       ok &= genDense(im1);
    
    //---- do odometry
    auto p_frmp = data_.p_frm_prev;
    if(p_frmp!=nullptr)
        odometry(*p_frmp, *p_frm);

    //--- write data
    if(cfg_.run.enWr)
        vod.wrData();

    //---- show
    if(cfg_.run.bShow)
        show();

    //---- save to previous frm
    data_.p_frm_prev = p_frm;
    return ok;
}
//-----------------
bool VOcv::triangulate(const FeatureMatchCv& fm,
                       vector<MPnt>& mpnts)const
{
    bool ok = true;
    stringstream s;

    auto& camc = cfg_.camc;
    mpnts.clear();
    // ( inner arry for each image)
    vector<cv::Point2d> Qs1, Qs2; // for calib triangulation
    //vector<cv::Point2d> Qs;// for sfm triangulation
    auto& ms = fm.FeatureMatch::data_.ms;
    int N = ms.size();
    for(auto& m : ms)
    {
        auto Q1 = ocv::toCv(m.p1);
        auto Q2 = ocv::toCv(m.p2);
        Qs1.push_back(Q1);
        Qs2.push_back(Q2);
    //    Qs.push_back(Q1);
    //    Qs.push_back(Q2);
    }
    //---- projection matrix P = K*T
    // We have 2 cameras.
    //  ( Note, projMat for cam coordinate, 
    //     so reverse transform)
    double b = cfg_.baseline;
    cv::Mat T1 = (cv::Mat_<double>(3,4) << 
            1, 0, 0,  b*0.5,
            0, 1, 0,  0,
            0, 0, 1,  0);
    cv::Mat T2 = (cv::Mat_<double>(3,4) << 
            1, 0, 0,  -b*0.5,
            0, 1, 0,  0,
            0, 0, 1,  0);

    cv::Mat K; cv::eigen2cv(camc.K, K); 
    cv::Mat Ps;
    cv::Mat P1 = K*T1;
    cv::Mat P2 = K*T2;

//   1) calib3d triangulation function
    cv::triangulatePoints(P1, P2, Qs1, Qs2, Ps);

 //  2) sfm triangulation function
 //   vector<cv::Mat> pmats;
 //   pmats.push_back(P1);
 //   pmats.push_back(P2);
 //   cv::sfm::triangulatePoints(Qs, pmats, Ps);
    s << "Triangulate pnts: " << N << endl;
    
    //---- De-homoge and fill triangulation result
    //s << "  Triangulate: " << endl;
    for(int i=0;i<N;i++)
    {
        vec4 h = ocv::toVec4(Ps.col(i));
        vec3 v; 
        if(!egn::normalize(h, v))
           v << 0,0,0;
        MPnt p;   
        p.mi = i;     
        p.Pt = cv::Point3f(v[0], v[1], v[2]);
        mpnts.push_back(p);
    //    s << "Pair:" << Qs1[i] << " | " << Qs2[i] << " => ";
      //  s << "(" << v.transpose() << ")" << endl;
    }
    log_d(s.str());
    return ok;
}


//-----------------
bool VOcv::odometry(const FrmCv& frm1,
                    const FrmCv& frm2)
{
    stringstream s;
    auto& odomc = cfg_.odom;
    //---- cam motion for left/right
    cv::Mat rL,rR,tL,tR;
    set<int> inliers;
    bool okL = solve_2d3d(frm1, frm2, true,  rL, tL, inliers);
    bool okR = solve_2d3d(frm1, frm2, false, rR, tR, inliers);
    if(!(okL | okR)) {
        log_e("stereo odometry failed");
        return false;
    }
    //---- Average or select
    cv::Mat rc = (okL && okR)? (rL + rR)*0.5 :
                okL ? rL : rR;
    cv::Mat tc =  (okL && okR)? (tL + tR)*0.5 :
                okL ? tL : tR;
    cv::Mat Rc;
    cv::Rodrigues(rc, Rc); 

    //------
    cv::Mat ec = rc * 180.0/M_PI; // to degree
    cv::Mat ec1, tc1; 
    cv::transpose(ec, ec1); cv::transpose(tc, tc1);
    s << "  Relative motion:  ";
    s << "ec=" << ec1 << ", tc=" << tc1 << endl; 
    // R/t is relative motion from frm1 to frm2
    //---- Update R/t global
    auto& odom = VO::data_.odom;
    mat3 Re; cv::cv2eigen(Rc, Re);
    vec3 te; cv::cv2eigen(tc, te);
    auto& Rw = odom.Rw;
    auto& tw = odom.tw;
    auto& ew = odom.ew;
    tw = tw + Rw * te;
    Rw = Rw * Re;
    //---- Euler pose
    cv::Mat Rwc, rwc;
    cv::eigen2cv(Rw, Rwc);
    cv::Rodrigues(Rwc, rwc); 
    cv::Mat ewc = rwc*180.0/M_PI; // to degree
    cv::cv2eigen(ewc, ew);
   
    //---- calc global points and fill
    /* TODO: result not good, obsolete
    auto& Pws = p_frm->Pws;
    calc_pnts(frm2, inliers, Pws);
    s << " calc points:" << Pws.size() << endl;
    //---
    */
    s << "  Global odom: ew=" << ew.transpose() 
        << ", tw=" << tw.transpose() << endl;
    log_d(s.str());

    return true;    
}

//-----------------
bool VOcv::solve_2d3d(const FrmCv& frm1,
                      const FrmCv& frm2,
                      bool bLeft,
                      cv::Mat& r, cv::Mat& t,
                      set<int>& inliers)const
{
    auto& odomc = cfg_.odom;

//  auto pm = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    auto& fm1 = *frm1.p_fm;
    auto& fm2 = *frm2.p_fm;

    FeatureMatchCv fmL, fmR;
    auto& fmd1 = fm1.data_;
    auto& fmd2 = fm2.data_;
    stringstream s;

    //---- 
    // mdL is match of L channel 
    // mdR is match of R channel
    FeatureMatchCv::MatchDt mdL, mdR;
    fmL.match(fmd1.fs1, fmd2.fs1, mdL);
    fmR.match(fmd1.fs2, fmd2.fs2, mdR);

    //---- to be filled
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;

    //---- Left odometry
  //  auto& i_mi = mdL.i1_mi;
    auto& md = bLeft?mdL:mdR;
    vector<int> mi_ary; // save index
    for(auto& m : md.dms)
    {
        int i1 = m.queryIdx; // fi frm1
        int i2 = m.trainIdx; // fi frm2
        //   search frm2 for 3d pnt
        MPnt mpnt;
        if(!frm2.find(i2, bLeft, mpnt))
            continue;
        // got mpnt is match pnt also
        //   of L/R in frm1.
        // Which has been triangulated.
        auto P = (odomc.mode==1)?
            mpnt.Pt : mpnt.Pd;
        if(P.z > odomc.z_TH)
            continue;
        mi_ary.push_back(mpnt.mi);
        pts_3d.push_back(P);
        // Find 2d pnt of previous frm
        auto& fmdQ = fmd1;
        auto& fs = bLeft?fmdQ.fs1 : fmdQ.fs2;
        auto Q = fs.pnts[i1].pt;
        pts_2d.push_back(Q);
    }
    //--- dbg
    int N = pts_2d.size();
    if(N < lcfg_.N_th_pnp)
        return false;

    //---- do solving
    //---- solve PnP
    cv::Mat inlrs;
    cv::Mat K; 
    cv::eigen2cv(cfg_.camc.K, K);
    if(!cv::solvePnPRansac(pts_3d, pts_2d, K, cv::Mat(), r, t, inlrs))
    {
        log_e("  solvePnPRansac() failed");
        return false;
    }
    //-------
    // ref : https://answers.opencv.org/question/196562/solvepnpransac-getting-inliers-from-the-2d-and-3d-points/
    int Ni = inlrs.rows;
    s << "  solvePnP() inliers: " << Ni << " of " << N << endl;
    //-------
    cv::Mat e = r*180.0/M_PI; // to degree
    cv::Mat e1,t1; 
    cv::transpose(e, e1); cv::transpose(t, t1);
    s << " cam " << (bLeft?"L":"R") << " motion: ";
    s << "e=" << e1 << ", t=" << t1 << endl; 
    // fill inliers
    for (int i = 0; i < inlrs.rows; i++)
    {
        int k = inlrs.at<int>(i, 0);
        cv::Point3f Pc = pts_3d[k];
        int mi = mi_ary[k];
        inliers.insert(mi);
        /*
        s << "  2d/3d:(" << pts_2d[k] 
            << ") -> ("  << pts_3d[k]  
            << ")" << endl; 
            */
    }
    // R/t is relative motion from frm1 to frm2
    log_d(s.str());
    return true;
    
}
//-----------
void VOcv::calc_pnts(const FrmCv& frmc,
                     const set<int>& mi_ary,
                     vec3s& Ps)const
{
    auto& odomc = cfg_.odom;
    auto& pcldc = cfg_.pntCloud;
    auto& odom = VO::data_.odom;

    auto& Rw = odom.Rw;
    auto& tw = odom.tw;
    //---- calc global points with inliers
    for(auto& mi : mi_ary)
    {
        MPnt mpnt;
        if(!frmc.at(mi, mpnt))
            continue;
        auto Pc = (odomc.mode==1)?
            mpnt.Pt : mpnt.Pd; 
        //--- Threshhold and check
        if(Pc.z > pcldc.z_TH)continue;
        if(Pc.z <0)continue;
        //----
        vec3 Pe; Pe << Pc.x, Pc.y, Pc.z;

        auto P = Rw*Pe + tw;
        Ps.push_back(P);
    }
}
/*
//-----------
bool VOcv::genDepth(const Img& im1,  
                    const Img& im2,
                    Dispar& depth)
{
    bool ok = true;
    //---- quasi slow and result not good
 // ok &= run_quasi(im1, im2, depth);
    ok &= depth.calc_dispar(cfg_.dispar, im1, im2);
    return ok;

}
*/
//----------------
bool VOcv::run_quasi(const Img& im1,
                     const Img& im2,
                     Dispar& depth)
{
    bool ok = true;

    ocv::ImgCv imc1(im1); cv::Mat imL = imc1.im_;
    ocv::ImgCv imc2(im2); cv::Mat imR = imc2.im_;
    
    cv::Size sz = imL.size();
    cv::Ptr<cv::stereo::QuasiDenseStereo> stereo = 
        cv::stereo::QuasiDenseStereo::create(sz);
    stereo->process(imL, imR);
    cv::Mat im_disp;
    im_disp = stereo->getDisparity();
    //cv::imshow("disparity map", disp);


    vector<cv::stereo::MatchQuasiDense> matches;
    stereo->getDenseMatches(matches);
    depth.p_im_disp = mkSp<ocv::ImgCv>(im_disp);
    return ok;
}

//------
/*
bool VOcv::genDense(const Img& imL)
{
    auto p_frmo = VO::data_.p_frm;
    if(p_frmo==nullptr) return false;
    auto& frmo = *p_frmo;
    auto& depth = frmo.depth;
    auto& pntc = frmo.pntc;

    //----
    auto& camc = cfg_.camc;
    CamCfg::Lense L; 
    if(!camc.toLense(L)) 
        return false;
    
    double b = cfg_.baseline;

    //----
    auto p_imd = depth.p_im_disp;
    if(p_imd==nullptr) return false;
    cv::Mat imd = ImgCv(*p_imd).im_;

    //----
    auto p_dense = mkSp<Points>();
    pntc.p_dense = p_dense;
    for(int v = 0; v<imd.rows; v++)
        for(int u = 0; u<imd.cols; u++)
        {
            double d = imd.at<float>(v,u);
            
            double z = d/b;
            double x= (u-L.cx)/L.fx;
            double y= (v-L.cy)/L.fy;
            x *= z;
            y *= z;
            if(d<0) 
                continue;
            //else std::cout << z <<" ";
            vec3 P; P << x,y,z;
            Color c;
            imL.get(Px(x,y), c);
            c = {255,255,255,255}; // debug
            p_dense->add({P,c});
        }
    
    //---- filter
    auto& fc = cfg_.pntCloud.filter;
    if(fc.en)
        p_dense->filter_stats(fc.meanK, fc.devTh);

    return true;
}

*/
//-----
void VOcv::show()
{
    auto p_frmo = VO::data_.p_frm;
    if(p_frmo==nullptr)
        return;
    auto& frmo = *p_frmo;
    auto& depth = frmo.depth;
    auto& pntc = frmo.pntc;
    //---- show disparity img
    auto p_imd = frmo.depth.p_im_disp;
    if(p_imd != nullptr)
    {
        auto& dc = cfg_.dispar;
        cv::Mat imd = ImgCv(*p_imd).raw(); 
        cv::Mat imdv;
        cv::ximgproc::getDisparityVis(imd, imdv, dc.vis_mul);
        imshow("Disparity", imdv);
    }

    //---- show points dense
    auto& pvis = VO::data_.pntVis;
    if(pvis.p_vis_dense==nullptr)
        pvis.p_vis_dense = Points::Vis::create();
    auto& vden = *pvis.p_vis_dense;
    vden.clear();
    auto p_dense = pntc.p_dense;
    if(p_dense!=nullptr)
    {
        vden.add(*p_dense, "dense", lcfg_.pnt_sz);
        vden.spin();
    }
    //---- show depth of point cloud
    

}
