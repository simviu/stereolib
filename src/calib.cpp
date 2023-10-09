
/*
   Author: Sherman Chen
   Create Time: 2023-10-09
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "stereolib/stereolibCv.h"

using namespace stereo;
//-----
namespace{

    class StereoCalib{
    public:
        struct Cfg{
            Sz sz_board{6,9};
        }; Cfg cfg_;
        //--- Calib Data for one cam
        struct CamSC{
            cv::Mat dist; // distortion
            cv::Mat Knew; // optimal new matrix

        };
        //---
        struct Data{
            CamSC cams[2];
        }; Data data_;
        bool calb_imgs(const string& sPath);
    protected:
        bool read_checkboard(const string& sPath);
    };
}
//-----
bool StereoCmd::run_stereo_calib(CStrs& args)
{
    KeyVals kvs(args);
    string spath = kvs["dir"];
    StereoCalib calib;
    return calib.calb_imgs(spath);
}

//-----
bool StereoCalib::calb_imgs(const string& sPath)
{
    bool ok = read_checkboard(sPath);
    return true;
}
//----
bool StereoCalib::read_checkboard(const string& sPath)
{  
        
    // Defining the dimensions of checkerboard
    int CHECKERBOARD[2]{6,9}; 
    
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;
    
    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;
    
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }
    
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imagesL, imagesR;
    // Path of the folder containing checkerboard images
    std::string pathL = "./data/left/*.png";
    std::string pathR = "./data/right/*.png";
    
    cv::glob(pathL, imagesL);
    cv::glob(pathR, imagesR);
    
    cv::Mat frameL, frameR, grayL, grayR;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
    bool successL, successR;
    
    // Looping over all the images in the directory
    for(int i{0}; i<imagesL.size(); i++)
    {
        frameL = cv::imread(imagesL[i]);
        cv::cvtColor(frameL,grayL,cv::COLOR_BGR2GRAY);
        
        frameR = cv::imread(imagesR[i]);
        cv::cvtColor(frameR,grayR,cv::COLOR_BGR2GRAY);
        
        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        successL = cv::findChessboardCorners(
            grayL,
            cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
            corner_ptsL);
            // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        
        successR = cv::findChessboardCorners(
            grayR,
            cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
            corner_ptsR);
            // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        /*
            * If desired number of corner are detected,
            * we refine the pixel coordinates and display 
            * them on the images of checker board
        */
        if((successL) && (successR))
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
        
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);
            cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);
        
            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsL,successL);
            cv::drawChessboardCorners(frameR, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsR,successR);
        
            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);
            imgpointsR.push_back(corner_ptsR);
        }
        
        cv::imshow("ImageL",frameL);
        cv::imshow("ImageR",frameR);
        cv::waitKey(0);
    }
    
    cv::destroyAllWindows();


    //---------
    cv::Mat mtxL,distL,R_L,T_L;
    cv::Mat mtxR,distR,R_R,T_R;
    cv::Mat Rot, Trns, Emat, Fmat;
    cv::Mat new_mtxL, new_mtxR;
    
    // Calibrating left camera
    cv::calibrateCamera(objpoints,
                        imgpointsL,
                        grayL.size(),
                        mtxL,
                        distL,
                        R_L,
                        T_L);
    
    new_mtxL = cv::getOptimalNewCameraMatrix(mtxL,
                                distL,
                                grayL.size(),
                                1,
                                grayL.size(),
                                0);
    
    // Calibrating right camera
    cv::calibrateCamera(objpoints,
                        imgpointsR,
                        grayR.size(),
                        mtxR,
                        distR,
                        R_R,
                        T_R);
    
    new_mtxR = cv::getOptimalNewCameraMatrix(mtxR,
                                distR,
                                grayR.size(),
                                    1,
                                    grayR.size(),
                                    0);
    //----
    auto& c0 = data_.cams[0];
    auto& c1 = data_.cams[1];
    c0.Knew = new_mtxL;
    c1.Knew = new_mtxR;
    c0.dist = distL;
    c1.dist = distR;
    return true;
}
//------