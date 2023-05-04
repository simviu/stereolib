/*
   Author: Sherman Chen
   Create Time: 2023-02-06
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "vsn/vsnLib.h"

namespace stereo
{
    using namespace vsn;


    //---- Multi-Cam cfg
    struct CamsCfg{
        string sName;
        struct OneCam{
            string sName;
            CamCfg camc;
            // Relative T from Left to this camera
            Pose T; 
        };
        vector<OneCam> cams;
        //--- cv data, virtual
        struct CvData{
          //  virtual Sp<Img> remap(Img& im, int cam_id)const = 0;
        };
        
        //--
        bool load(const string& sf);
        auto get_cvd()const { return p_cv_data; }
    protected:
        Sp<CvData> p_cv_data= nullptr;
        bool init_rectify();
        string str()const;
    };

    //---- SGBM cfg
    struct SGBM{
        int  	minDisparity = 0;
        int  	numDisparities = 16;
        int  	blockSize = 3;
        int  	P1 = 0;
        int  	P2 = 0;
        int  	disp12MaxDiff = 0;
        int  	preFilterCap = 0;
        int  	uniquenessRatio = 0;
        int  	speckleWindowSize = 0;
        int  	speckleRange = 0;
        //--- disparity WSL filter
        struct WLSFilter{
            bool en=true;
            float lambda=8000;
            float sigma=1;
        }; WLSFilter wls_filter;
    };  // TODO: move to VOcv
    //---- disparity cfg
    struct DisparityCfg{
        
        SGBM sgbm;
        float vis_mul = 8.0;
        bool load(const string& sf);
    };
    
    //Stereo video odometry
    class VO {
    public:
        //VO(){ init_cmds(); }
        static Sp<VO> create();
        static Sp<VO> create_os3(
            const string& sf_voc,    // VOC txt files
            const string& sf_cfg     // Yaml cfg of ORB-SLAM
            );
        
        //----
        struct Cfg{
            double fps = 30.0;
            CamCfg camc;
        //  bool bShow = false;
            double baseline = 0.50;
            struct Odom{
                // 1:triangulation , 2:depth
                int mode=1;
                // z threshold
                double z_TH = 50;
            }; Odom odom;
            struct Feature{
                int Nf = 100;
            }; Feature feature;

            DisparityCfg dispar;

            struct Run{
                bool bShow=false;
                bool enDense = false;
                bool enDepth = false;
                bool enWr = false;
            }; Run run;

            struct PointCloud{
                double z_TH = 40;
                struct Filter{
                    bool en = true;
                    float meanK = 50;
                    float devTh = 1.0;
                    float voxel_res = 0.03; 
                }; Filter filter;
            }; PointCloud pntCloud;

            //---- omnidirectional
            struct Omni{
                // factory
                static Sp<Omni> load(const string& sf); 
            }; Sp<Omni> p_omni = nullptr;

            //----
            bool load(const string& sf);
            string str()const;

        };
        Cfg cfg_;
        
        //---- Frm data
        struct Frm{
            // Triangulated feature points
            //  in global space.
            vec3s Pws; 
            
            //---- point cloud
            struct PntCloud{
                Sp<Points> p_dense  = nullptr;
                Sp<Points> p_sparse = nullptr;
            }; PntCloud pntc;

            Sp<Img> p_im_disp = nullptr;
        };
        //----
        struct Data{
            int frmIdx = 0;
            double t = 0;
            //----
            struct Wr{
                ofstream ofs_pnts_spar;
                ofstream ofs_pnts_dense;
                ofstream ofs_Tw;
                bool open();
                void close();
            }; Wr wr;
            // local points by stereo matching 
            //   and triangulations.
            //---
            struct Odom{
                Odom(){ reset(); }
                mat3 Rw;
                vec3 tw;
                vec3 ew; // euler angle
                void reset()
                { Rw = mat3::Identity(); tw << 0,0,0; ew << 0,0,0; }
            }; Odom odom;
            
            //---- current Frm result
            Sp<Frm> p_frm = nullptr;

            //---- vis
            struct PntVis{

                Sp<Points::Vis> p_vis_dense = nullptr;
            }; PntVis pntVis;

            // wr data
            bool wrData();
            void close(){ wr.close(); }
        };
        //----
        virtual bool onImg(const Img& im1, 
                        const Img& im2)=0;
        /*
        virtual bool genDepth(const Img& im1,  
                            const Img& im2,
                            Dispar& dspr)=0;
        */
        //--- save trajectory
        virtual void onClose(){};
        virtual bool save(const string& sf)=0;
        //----
        auto& getData()const{ return data_; }
        void onFinish(){ data_.close(); }
        void setFrmIdx(int i){ data_.frmIdx=i; }
    //   void showLoop();
    protected:
        Data data_;
    }; // VO

    //----
    class VO_mng : public Cmd{
    public:
        VO_mng(){ init_cmds(); }
    protected:
        void init_cmds();

        Sp<VO> p_vo_ = nullptr;
        bool init(CStrs& args);
        bool init_os3(CStrs& args);
        bool run_frms(CStrs& args);
        bool run_video(CStrs& args);
        bool run_dualCams(CStrs& args);

        bool chk_init()const;
    };
    
    //------------
    // DepthGen
    //------------
    //---- Re-construct 3d point cloud scene
    class DepthGen : public Cmd{
    public:
        struct Cfg{
            CamsCfg cams;
            DisparityCfg disp;
            struct Imgs{
                vector<string> sDirs{"L","R","C","D","N"};                
                struct Idxs{
                    int color=0;
                    int dispar=-1;
                    int depth=-1; 
                }; Idxs idxs;
            };Imgs imgs;

            struct DepthC{ 
                Rng<double> range; // box range
            }; DepthC depth;
            //----
            struct VisCfg{
                Points::Vis::Cfg pntvc;
            }; VisCfg visc;
            //----
            string s_wdir = "./out/";
            //----
            bool load(const string& sf);
            //--- run options
            bool b_save_pcd = false;
            bool set(const KeyVals& kvs);
        }; Cfg cfg_;
        //----
        struct Frm{
            Frm(int i):idx(i){}
            int idx=0;
            vector<Sp<Img>> imgs;
            Pose T;
            Points pnts;

            virtual bool calc(const Cfg& cfg)=0;
            static Sp<Frm> create(int i);

            bool recon(const Cfg& cfg);
            bool load(const Cfg& cfg, const string& sPath, int i);
            bool load_imgs(const Cfg& cfg, const string& sPath, int i);
            bool genPnts(const Cfg& cfg);
        //  bool renderPnts(const Cfg& cfg);
            bool rectify(const CamsCfg& camcs);
            //----
            struct Data{
                Sp<Img> p_im_disp = nullptr; 
                Sp<Img> p_im_depth = nullptr;
                
                //--- undistorted, 0,1,2 -> L,R,C
                vector<Sp<Img>> ud_imgs;
            };
            auto& data()const{ return data_; }
            bool load(Video& vid);
        protected:
            Data data_;

            Sp<Img> calc_dispar(const DisparityCfg& cfg,
                            const Img& im1,
                            const Img& im2)const;            
            bool genPnts_byDepth(const Cfg& cfg);
            bool genPnts_byDisp(const Cfg& cfg);
            bool genPnts_byLR(const Cfg& cfg);
            void disp_to_pnts(const Cfg& cfg);
        };

        DepthGen(){ init_cmds(); }
        bool onImg(Frm& frm);
        bool run_video(CStrs& args);
        bool run_frm(CStrs& args);
        bool run_frms(CStrs& args);
    protected:
        void init_cmds();

        //----
        struct Data{
            Points pnts; // all points
            Sp<Points::Vis> p_pvis_frm = nullptr; 
        }; Data data_;
        Sp<Points::Vis> get_frm_pnt_vis();

        void show(const Frm& f);
    };
    
    //-------------
    // ReconScn
    //-------------
    class ReconScn : public Cmd
    {
    public:
        ReconScn(){ init_cmds(); }
        struct Cfg{
            bool load(const string& sf);
        }; Cfg cfg_;

        //----
        struct Traj{
            bool load(const string& sf);
            struct TPnt{ // traj point
                Pose T;
                double t=0;
            }; 
            vector<TPnt> tpnts;  
        private:
        };
        //-----
        struct Data{
            Traj traj;
            Points pnts;
        };
    protected:
        Data data_;

        void init_cmds();
        bool init(CStrs& args);
        bool run_merge(CStrs& args);
    };

    //-----
    class StereoCmd : public Cmd
    {
    public:
        StereoCmd(){ init_cmds(); }
    protected:
        void init_cmds();
        bool init(CStrs& args);
       
    };
}
