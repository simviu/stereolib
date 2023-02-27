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
    //---- Depth
    class Depth{
    public:
        
        //---- depth disparity map
        Sp<Img> p_im_disp = nullptr;
         // depth img
        Sp<Img> p_im_depth = nullptr;

        bool calc_dispar(const DisparityCfg& cfg,
                         const Img& im1,
                         const Img& im2);
    };

    //Stereo video odometry
    class VO{
    public:

        static Sp<VO> create();
        
        //----
        struct Cfg{
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
            //---- Depth
            Depth depth;
            
            //---- point cloud
            struct PntCloud{
                Sp<Points> p_dense  = nullptr;
                Sp<Points> p_sparse = nullptr;
            }; PntCloud pntc;
        };
        //----
        struct Data{
            int frmIdx = 0;
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

        virtual bool genDepth(const Img& im1,  
                            const Img& im2,
                            Depth& depth)=0;

        auto& getData()const{ return data_; }
        void onFinish(){ data_.close(); }
        void setFrmIdx(int i){ data_.frmIdx=i; }
    //   void showLoop();
    protected:

        Data data_;
    }; // VO

    //------------
    // Recon3d
    //------------

    //---- Re-construct 3d point cloud scene
    class Recon3d : public Cmd{
    public:
        struct Cfg{
            CamsCfg cams;
            DisparityCfg disp;
            struct FrmsCfg{
                vector<string> sDirs{"L","R","C","D","N"};                
                int color_img = 2;
                int dispar_img = -1;
                int depth_img = -1; // TODO: json
            }; FrmsCfg frms;
            struct DepthC{ 
                Rng<float> range; // box range
            }; DepthC depth;

            bool load(const string& sf);
        }; Cfg cfg_;
        //----
        struct Frm{
            vector<Sp<Img>> imgs;
            Pose T;
            Points pnts;
            Depth depth;

            virtual bool calc(const Cfg& cfg)=0;
            static Sp<Frm> create();

            bool recon(const Cfg& cfg);
            bool load(const Cfg& cfg, const string& sPath, int i);
            bool load_imgs(const Cfg& cfg, const string& sPath, int i);
            bool genPnts(const Cfg& cfg);
            bool renderPnts(const Cfg& cfg);
            bool rectify(const CamsCfg& camcs);
            //----
            struct Data{
                Sp<Img> p_im_disp = nullptr; 
                Sp<Img> p_im_depth = nullptr;
                
                //--- undistorted, 0,1,2 -> L,R,C
                vector<Sp<Img>> ud_imgs;
            };
            auto& data()const{ return data_; }
            
        protected:
            Data data_;

            bool genPnts_byDepth(const Cfg& cfg);
            bool genPnts_byDisp(const Cfg& cfg);
            bool genPnts_byLR(const Cfg& cfg);
        };

        Recon3d(){ init_cmds(); }
        bool onImg(Frm& frm);
        bool run_frms(const string& sPath);
        bool run_frm(const string& sPath, int i);
    protected:
        void init_cmds();

        //----
        struct Data{
            // local point cloud
            Sp<Points::Vis> p_pvis_frm = 
                Points::Vis::create();
        }; Data data_;
        void show(const Frm& f);

    };

    //-----
    class StereoCmd : public Cmd
    {
    public:
        StereoCmd(){ init_cmds(); }
    protected:
        void init_cmds();
        bool init(CStrs& args);
        Sp<stereo::Recon3d> p_recon_ = 
            mkSp<stereo::Recon3d>();
    };
}
