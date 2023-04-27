/*
   Author: Sherman Chen
   Create Time: 2023-01-20
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"
#include "json/json.h"
#include "vsn/vsnLibCv.h"

using namespace stereo;

namespace{
    struct LCfg{
        float fps = 30; 
        string s_pcds = "pcds/";
        string s_gpcd = "global_pnts.pcd";
    }; LCfg lc_;
    
}
//----
bool ReconScn::Cfg::load(const string& sf)
{
    return true;
}
//----
void ReconScn::init_cmds()
{
    sHelp_ = "(Recon 3d Scene of dense point cloud)";

    Cmd::add("init", mkSp<Cmd>("cfg=<CFG_FILE>",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        return cfg_.load(lookup(kv, "cfg")); 
    }));

    Cmd::add("pcds", mkSp<Cmd>("dir=<DIR> traj=<TRAJ_FILE> (Recon by frm pcd and traj file)",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        string sdir = lookup(kv, "dir"); 
        return run_pcds(sdir); 
    }));
}

//----
bool ReconScn::run_pcds(const string& sdir)
{

    return true;
}
//----
bool ReconScn::Traj::load(const string& sf)
{
    ifstream ifs(sf);
    if(!ifs.is_open())
        { log_ef(sf); return false; }
    //---
    bool ok = true;
    int i=0;
    while(!ifs.eof())
    {
        i++;
        string sln;
        getline(ifs, sln);
        ok &= !ifs.fail();
        if(!ok)break;
    }
    //---
    if(!ok)
    {
        log_e("Error reading line:"+to_string(i));
        return false;
    }
    return true;
}

