/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/ocv_hlpr.h"
#include <filesystem>

using namespace stereo;
using namespace test;
using namespace cv;

namespace
{
    const struct
    {   
        string sf_camc    = "cfg/cam.yml";
        string sf_stereoc = "cfg/stereo.json";
        string sf_seqL    = "seq/image_0";
        string sf_seqR    = "seq/image_1";

    } lcfg_;
    //--------------
    //-------- get index from Kitti img filename
    int getIdx(const string& sf)
    {
        string s= std::filesystem::path(sf).stem();
        for(int i=0;i<s.length()-1;i++)
        {
            if(s[i]!='0') break;
            s[i] = ' ';
        }
        
        int i = std::stoi(s);
        return i;
    }

}
//--------------------------
bool TestStereo::testKittyGray()const
{
    bool ok = true;
    log_i("run TestStereo()...");
    vector<string> sfLs, sfRs;
    cv::glob(lcfg_.sf_seqL, sfLs);
    cv::glob(lcfg_.sf_seqR, sfRs);
    CamCfg camc;
    if (!camc.load(lcfg_.sf_camc))
        return false;
    //---- load stereo cfg
    //---- stereo VO test
    auto p_vo = VO::create();
    auto &vo = *p_vo;
    if (!vo.cfg_.load(lcfg_.sf_stereoc))
        return false;

    vo.cfg_.camc = camc;

    //---- main loop
    int N = sfLs.size();
    if (N > sfRs.size())
        N = sfRs.size();
    log_i("seq has total imgs:" + to_string(N));

    //-----
    for (int i = 0; i < N; i++)
    {
        //---- load image
        int lf = cv::IMREAD_GRAYSCALE;

        auto p1 = Img::loadFile(sfLs[i], lf);
        auto p2 = Img::loadFile(sfRs[i], lf);
        if ((p1 == nullptr) || (p2 == nullptr))
        {
            log_e("Failed load image '" +
                  sfLs[i] + "' or '" + sfRs[i] + "'");
            return false;
        }
        auto &im1 = *p1;
        auto &im2 = *p2;

        //--- process img
        int fi = getIdx(sfLs[i]);
        vo.setFrmIdx(fi);
        vo.onImg(im1, im2);

        //--- log Kitti Tw to file
        /*
        auto& odom = vo.getData().odom;
        cv::Mat Rw, tw;
        cv::eigen2cv(odom.Rw, Rw);
        cv::eigen2cv(odom.tw, tw);
        int idx = getIdx(sfLs[i]);
        oftw << kitti_line(Rw, tw, idx);
        */
        //--- write points
        /*
        auto p_frm = vo.getData().p_frm;
        if(p_frm!=nullptr)
            for(auto& P : p_frm->Pws)
                ofps << P.x() << " " << P.y() << " " << P.z() << endl;
        */
        //------ Press  ESC on keyboard to exit
        char c = (char)cv::waitKey(25);
        if (c == 27)
            break;
    }

    // Closes all the frames
    cv::destroyAllWindows();
    vo.onFinish();
    //ofps.close();
    //oftw.close();
    return ok;
}

//----------
bool TestStereo::run()
{
  //  return testKittyGray();    
    return test_imgLR();    
}

