/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolib.h"



using namespace stereo;
//----
namespace{
    using OmniCfg = VO::Cfg::Omni;
    //---- impl
    struct OmniCfgImp : public OmniCfg{
        bool loadFile(const string& sf)
        {
            return true;
        }
    protected:
    };
}


//----------------
//---- factory
Sp<OmniCfg> OmniCfg::load(const string& sf)
{  
    auto p = mkSp<OmniCfgImp>();
    if(p->loadFile(sf)) return p;
    return nullptr; 
} 


