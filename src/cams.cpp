/*
   Author: Sherman Chen
   Create Time: 2023-02-23
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"
#include "json/json.h"


using namespace stereo;

//-----------
string CamsCfg::str()const
{
    stringstream s;
    s << sName <<":" << endl;
    int i=0;
    for(auto& c : cams)
    {
        s << "  cam "+to_string(i++)+
                " '" << c.sName << "': ";
        s << "  T=" << c.T.str() << "\n";
    }

    return s.str();
}

//-----------
bool CamsCfg::load(const string& sf)
{

    log_i("Load Multi-Cam cfg :'"+sf+"'");
    ifstream ifs(sf);
    sys::FPath fp(sf);

    if(!ifs)
    {
        log_ef(sf);
        return false;
    }
    bool ok = true;
    //----
    try{

        Json::Reader rdr;
        Json::Value jd;
        rdr.parse(ifs, jd);
        sName = jd["name"].asString();
        auto& jcs = jd["cams"];
        for(auto& jc : jcs)
        {
            OneCam oc;
            oc.sName = jc["name"].asString();
            ok &= s2v(jc["pos"].asString(), oc.T.t); 
            ok &= s2q(jc["quat"].asString(), oc.T.q);
            string sfc = fp.path + jc["cfg"].asString();
            if(!oc.camc.load(sfc))
            {   log_ef(sfc); return false; }
            cams.push_back(oc); 
        }
    }
    catch(exception& e)
    {
        log_e("exception caught:"+string(e.what()));
        return false;
    }
    
    if(!ok)
    {
        log_e("CamsCfg::load() json error");
        return false;
    }    
    //---- dbg
    string s = this->str();
    log_d(s);

    return init_rectify();
}

//----
bool CamsCfg::init_rectify()
{
    if(cams.size()<2)
    {
        log_e("need at least 2 cameras");
        return false;
    }
    //----
    struct Cd{ cv::Mat K,D,Ro,P,map1, map2; };
    Cd cd[2];
    for(int i=0;i<2;i++)
    {
        auto& cc = cams[i].camc;
        cv::eigen2cv(cc.K, cd[i].K);
        cv::eigen2cv(cc.D, cd[i].D);
    }
    
    //----
    auto& cc0 = cams[0].camc;
    auto& cc1 = cams[1].camc;
    auto sz = cc0.sz;
    assert(sz.w == cc1.sz.w);
    assert(sz.h == cc1.sz.h);
    cv::Size imsz(sz.w, sz.h);

    //----
    cv::Mat R,t,Q;
    cv::eigen2cv(mat3(cams[1].T.q), R);
    cv::eigen2cv(cams[1].T.t, t);
    //---
    cv::stereoRectify(cd[0].K, cd[0].D, 
                      cd[1].K, cd[1].D,
                      imsz, R, t, 
                      cd[0].Ro, cd[1].Ro, 
                      cd[0].P,  cd[1].P, Q);
    //--- fill remap map1/map2 for undistortion
    auto p = mkSp<CamsCfgCvd>();
    p_cv_data = p;
    auto& cvd = *p;
    for(int i=0;i<2;i++)
    {
        auto& d = cd[i];
        CamsCfgCvd::RemapD rmd;
        cv::initUndistortRectifyMap(
                d.K, d.D, d.Ro, d.P, imsz, CV_32FC1,
                rmd.map1, rmd.map2);
        cvd.remapds.push_back(rmd);
        
    }
    //--- fill Q mat for reproj 3d
    int tpQ = Q.type();
    cvd.Q = Q;
    //----
    return true;
}

//---
Sp<Img> CamsCfgCvd::remap(const Img& im, int cam_id)const 
{
    assert(cam_id < remapds.size());
    auto& rmd = remapds[cam_id];

    auto p = mkSp<ImgCv>();
    cv::Mat imc = img2cv(im);
    cv::Mat imr;
    cv::remap(imc, p->im_, rmd.map1, rmd.map2, 
                    cv::INTER_LANCZOS4);
    return p;
}
