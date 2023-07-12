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
    
    //---- PFM is image format support float point
    bool use_pfm(const string& s)
    { 
        const set<string> pfm_imgs
            {"disp", "depth", "dispConf", "depthConf"};
        return pfm_imgs.find(s)!=pfm_imgs.end(); 
    } 
    
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
        
        int flag = -1 ; // unchange
        Sp<Img> p = nullptr;

        //---- auto detect if pfm needed.
        if(use_pfm(sdir))
        {
            string sf = sPath + "/"+sdir+"/"+si+".pfm";
            cv::Mat im = loadPFM(sf);
            p = mkSp<ImgCv>(im);
        }
        else // normal image, png
        {
            string sf = sPath + "/"+sdir+"/"+si+".png";
            p =Img::loadFile(sf, flag);  
        }      
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
