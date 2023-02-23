/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibTest.h"

using namespace rov;
using namespace ut;
using namespace test;
using namespace cv;
using namespace vsn;
using namespace stereo;

namespace{
    const struct{
        string sf_camc = "testd/camc/cam.yml";
        string sf_voc = "testd/camc/stereo.json";
        string sf_cap = "testd/cur.mkv";
        bool enVWr = false; // video LR wr
        bool enIWr = false; // Img LR wr
        bool enVO = true;
        string sf_wrL = "L.avi";
        string sf_wrR = "R.avi";

        //---- zed cfg
        struct VideoCfg{
            int w = 952;
            int h = 535;
            int dw = 8;
            int dh = 243; 
        };
        VideoCfg vc_zed1{952,535,8,243};//  zed office1 video
        VideoCfg vc_zed2{952,535,8,215};//  zed ChArUco video
        VideoCfg vc_webcam{640,362,0,216};//  zed ChArUco video
        // dbg
        struct Dbg{
            set<int> wr_frms{1, 800, 400};
        }; Dbg dbg;
    }lcfg_;
}

//--------------------------
bool TestVO::test_video()
{
    bool ok = true;
    log_i("run TestVO...");   
    CamCfg camc;
    if(!camc.load(lcfg_.sf_camc))
        return false;
    //----
    //cv::VideoCapture cap(lcfg_.sf_cap); 
    auto p_vr = vsn::Video::open(lcfg_.sf_cap);
    if(p_vr==nullptr)
    {
        log_ef(lcfg_.sf_cap);
        return false;
    }
    auto& vr = *p_vr;
    float fps = vr.cfg_.fps;

    // Check if camera opened successfully
    //if(!cap.isOpened()){
    //    cout << "Error opening video stream or file" << endl;
    //    return -1;
    //}
    //float fps = cap.get(CAP_PROP_FPS);//Getting the total number of frames//
    //cout << "  FPS:" << fps << endl;
    //----------
    auto& vc = lcfg_.vc_webcam;
    auto& w = vc.w;
    auto& h = vc.h;
    auto& dw = vc.dw;
    auto& dh = vc.dh;
    //----------
    Sp<vsn::Video> p_vwrL = nullptr;
    Sp<vsn::Video> p_vwrR = nullptr;
    if(lcfg_.enVWr)
    {
        Video::Cfg vc; 
        vc.fps = fps;
        vc.sz = {w,h};
        p_vwrL = Video::create(lcfg_.sf_wrL, vc);
        p_vwrR = Video::create(lcfg_.sf_wrR, vc);
    }
    //---- stereo VO test
    auto p_vo = VO::create();
    auto& vo = *p_vo;
    auto& voc = vo.cfg_;
    if(!voc.load(lcfg_.sf_voc))
    {
        log_ef(lcfg_.sf_voc);
        return false;
    }
    voc.camc = camc;


    //-------------
    int fi=0;
    while(1){
        fi++;

        auto p_im = vr.read();
        if(p_im==nullptr) break;
        vsn::ImgCv im(*p_im);
        cv::Mat frame = im.im_;
        // Capture frame-by-frame
        //cap >> frame;
    
        // If the frame is empty, break immediately
        if (frame.empty())
            break;
        //---- split and crop
        //cv::Mat im1 = frame(rang(), range());
        Mat imci1 = frame({dh, h+dh},{dw, dw+w});
        Mat imci2 = frame({dh, h+dh},{dw+ w, dw+w*2});
        //---- to gray
//      Mat imc1, imc2;
//      cvtColor(imci1, imc1, cv::COLOR_RGB2GRAY);
//      cvtColor(imci2, imc2, cv::COLOR_RGB2GRAY);
        //----- save frm
        vsn::ImgCv imwL(imci1);
        vsn::ImgCv imwR(imci2);
        if(lcfg_.enVWr)
        {
            p_vwrL->write(imwL);
            p_vwrR->write(imwR);
        }
        //----
        if(lcfg_.enIWr)
        {
            // wr L/R frm
            auto& wr_frms = lcfg_.dbg.wr_frms;
            if(wr_frms.find(fi)!=wr_frms.end())
            {
                string sfw = std::to_string(fi);
                imwL.save(sfw + "L.png");
                imwR.save(sfw + "R.png");
            }
        }
        //--- dbg
        cv::rectangle(frame, {dw,dh}, {dw+w-1, h + dh-1},
                    Scalar(0, 0, 250),
                    2, LINE_8);
        //----
        // Display the resulting frame
        cv::imshow( "Frame", frame );
       // cv::imshow( "Left", im1 );
       // cv::imshow( "Right", im2 );

        auto p_im1 = mkSp<ocv::ImgCv>(imci1);
        auto p_im2 = mkSp<ocv::ImgCv>(imci2);
        auto& im1 = *p_im1;
        auto& im2 = *p_im2;

        //--- run VO
        if(lcfg_.enVO)
            vo.onImg(im1, im2);

        //------ Press  ESC on keyboard to exit
        char c=(char)cv::waitKey(25);
        if(c==27)
        break;
    }
    
    // When everything done, release the video capture object
    //cap.release();
    vr.close();
 

    // Closes all the frames
    cv::destroyAllWindows();
        
    return ok;

}





