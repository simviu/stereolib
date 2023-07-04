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
        // PFM is image format support float point
        set<string> pfm_imgs{"disp", "depth", "depthConf"};
        bool use_pfm(const string& s)const
        { return pfm_imgs.find(s)!=pfm_imgs.end(); }        
    }; LCfg lc_;
}


//----
Sp<Img> DepthGen::Frm::findImg(const string& s)const
{
    auto it = imgs.find(s);
    if(it==imgs.end()) return nullptr;
    return it->second;
}

//-----
bool DepthGen::Frm::load(Video& vid)
{
    auto p = vid.read();
    if(p==nullptr)return false;
    auto& im = *p;
    Sz sz = vid.cfg_.sz;
    sz.w *= 0.5;

    //---
    CStrs sdirs{"left","right"};
    for(int i=0;i<2;i++)
    {
        Px c(sz.w*(i+0.5), sz.h*0.5);
        Rect r(c, sz);
        auto pi = p->crop(r);
        if(pi==nullptr)return false;
        imgs[sdirs[i]]= pi;
    }
    return true;
}

//-----

bool DepthGen::Frm::load_imgs(const Cfg& cfg, const string& sPath, int i)
{
    string si = to_string(i);
    auto& sDirs = cfg.imgs.sDirs;
    int N = sDirs.size();
    int k=0;
    for(k=0;k<N;k++)
    {
        string sdir = sDirs[k];
        /*
        int flag = (k==cfg.frms.color_img) ?  cv::IMREAD_COLOR :
                   (k==cfg.frms.depth_img) ?  cv::IMREAD_ANYDEPTH :
                   cv::IMREAD_GRAYSCALE;
                   */
        int flag = -1 ; // unchange
        auto p = Img::loadFile(sPath + "/"+sdir+"/"+si+".png", flag);        
        if(p==nullptr) break;
        imgs[sdir] = p;

    }
    if(k<N)
    {
        log_e("not all img loaded OK in path:'"+sPath+"'");
        return false;
    }
    
    return true;

}
//----
bool DepthGen::Frm::load(const Cfg& cfg, const string& sPath, int i)
{
    bool ok = true;
    ok &= load_imgs(cfg, sPath, i);
    return ok;
}
