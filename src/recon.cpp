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

    Cmd::add("merge", mkSp<Cmd>("pcds=<DIR> traj=<TRAJ_FILE> (Recon by frm pcd and traj file)",
    [&](CStrs& args)->bool{ 
       
        return run_merge(args); 
    }));
}


//----
bool ReconScn::Traj::load(const string& sf)
{

    ifstream ifs(sf);
    if(!ifs.is_open())
        { log_ef(sf); return false; }
    log_i("loading traj file:'"+sf+"'...");
    //---
    bool ok = true;
    int i=0;
    try{
        while(!ifs.eof())
        {
            i++;
            string sln;
            getline(ifs, sln);
            if(ifs.fail())continue;
            //-----
            TPnt tp; 
            vector<double> ds;
            ok &= s2data(sln, ds, ' ');
            if(ds.size()==0)continue;
            if(ds.size()<8) 
                throw ErrExcept("dec_Kitti() expect 8 digit each line"); 
            tp.t = ds[0];
            tp.T.t << ds[1], ds[2], ds[3];
            // quat w,x,y,z
            tp.T.q = quat(ds[7],ds[4],ds[5],ds[6]);
            tpnts.push_back(tp);
            
        }
        //----
        if(i==0) 
            throw ErrExcept(" empty traj file");
    }
    catch(ErrExcept& e)
    {
        log_e(e.str());
        return false;
    }
    //---
    log_i("  load OK, N="+to_string(tpnts.size()));
    return true;
}


//----
bool ReconScn::run_merge(CStrs& args)
{
    KeyVals kvs(args);
    string sd_pcds = kvs["pcds"]; 
    string sf_traj = kvs["traj"];
    if(sd_pcds=="" || sf_traj=="")return false; 
    int i=1;
    //---- Load Traj
    if(!data_.traj.load(sf_traj))
        return false;

    //---- load PCDs
    while(1)
    {
        string sf = sd_pcds +"/" + to_string(i)+".pcd";
    }
    return true;
}
