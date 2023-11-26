
/*
   Author: Sherman Chen
   Create Time: 2023-10-09
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

// ref : https://learnopencv.com/making-a-low-cost-stereo-camera-using-opencv/

#include "stereolib/stereolibCv.h"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>

using namespace stereo;
using namespace cv;
//-----
namespace{
    void draw_det(Mat im, 
            vector<int> markerIds,
            vector<vector<Point2f> > markerCorners)
    {
        for(auto& mid : markerIds)
            for(auto& mc : markerCorners)
            {
                for(int i=0;i<4;i++) // 4 points
                    putText(im, to_string(i), mc[i],
                        FONT_HERSHEY_COMPLEX, 
                        1,Scalar(200,0,0), 2);
            }
    }


}

//-----
bool StereoCalib::calb_imgs(const string& sPath)
{
    bool ok = read_imgs_charuco(sPath);
    ok &= calc_stereo();
    ok &= calc_rectify();
    return ok;
}
//----
bool StereoCalib::read_imgs(const string& sPath)
{
    return true;
}
//----
bool StereoCalib::read_imgs_charuco(const string& sPath)
{
    //---- expect fixed 5x7 ChArUco of DICT_6x6_250.
    int N_markers = 17; 

    //----
    auto& objpoints = data_.objpoints;
    auto& imgpointsL = data_.imgpointsL;
    auto& imgpointsR = data_.imgpointsR;
    //----
    // Defining the world coordinates for 
    // one frame of 3D points
    /*
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<szb.h; i++)
        for(int j{0}; j<szb.w; j++)
            objp.push_back(cv::Point3f(j,i,0));
    */
    //-----
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imsL, imsR;
    cv::glob(sPath + "/left/*.png",  imsL);
    cv::glob(sPath + "/right/*.png", imsR);

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = new cv::aruco::CharucoBoard(cv::Size(5, 7), 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();

    for(int i{0}; i<imsL.size(); i++)
    {
        log_i("read frm:"+to_string(i));
        cv::Mat imL = cv::imread(imsL[i]);//, cv::IMREAD_GRAYSCALE);
        cv::Mat imR = cv::imread(imsR[i]);//, cv::IMREAD_GRAYSCALE);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;

        cv::aruco::detectMarkers(imL, cv::makePtr<cv::aruco::Dictionary>(board->getDictionary()), 
            markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() != N_markers) continue;
        //----            
        cv::aruco::drawDetectedMarkers(imL, 
            markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        
        //-----
        //cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, 
        //    charucoCorners, charucoIds, cameraMatrix, distCoeffs);
        //----
        draw_det(imL, markerIds, markerCorners);
        //-----
        cv::imshow("imL", imL);
        if((char)cv::waitKey(1) == 27) break;
    }
    return true;  
}
//----
bool StereoCalib::read_imgs_checkboard(const string& sPath)
{  
        
    // Defining the dimensions of checkerboard
    //int CHECKERBOARD[2]{6,9}; 
    auto& szb = cfg_.sz_board;
    // dbg
    //szb = {9,9};
    // Creating vector to store vectors of 3D points for each checkerboard image
    //std::vector<std::vector<cv::Point3f> > objpoints;
    auto& objpoints = data_.objpoints;
    // Creating vector to store vectors of 2D points for each checkerboard image
    //std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;
    auto& imgpointsL = data_.imgpointsL;
    auto& imgpointsR = data_.imgpointsR;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<szb.h; i++)
    {
    for(int j{0}; j<szb.w; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }
    
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imagesL, imagesR;
    // Path of the folder containing checkerboard images
    std::string pathL = sPath + "/left/*.png";
    std::string pathR = sPath + "/right/*.png";
    
    cv::glob(pathL, imagesL);
    cv::glob(pathR, imagesR);
    
    cv::Mat frameL, frameR, grayL, grayR;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
    bool successL, successR;
    
    cv::Size szbc(szb.w, szb.h);
    // Looping over all the images in the directory
    int j=0;
    for(int i{0}; i<imagesL.size(); i++)
    {
        log_i("read frm:"+to_string(i));
        frameL = cv::imread(imagesL[i], cv::IMREAD_GRAYSCALE);
        int dt = frameL.type();
        grayL = frameL;
        if(grayL.channels()==3)
            cv::cvtColor(frameL,grayL,cv::COLOR_BGR2GRAY);
        data_.sz_img = grayL.size();

        frameR = cv::imread(imagesR[i], cv::IMREAD_GRAYSCALE);
        grayR = frameR;
        if(grayR.channels()==3)
            cv::cvtColor(frameR,grayR,cv::COLOR_BGR2GRAY);
        
        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        successL = cv::findChessboardCorners(
            grayL,
            szbc,
            corner_ptsL,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        
        successR = cv::findChessboardCorners(
            grayR,
            szbc,
            corner_ptsR,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        /*
            * If desired number of corner are detected,
            * we refine the pixel coordinates and display 
            * them on the images of checker board
        */
        if((successL) && (successR))
        {
            log_i("  found board");
            j++;
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
        
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);
            cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);
        
            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frameL, szbc, corner_ptsL,successL);
            cv::drawChessboardCorners(frameR, szbc, corner_ptsR,successR);
        
            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);
            imgpointsR.push_back(corner_ptsR);
        }
        //---
        cv::imshow("grayL",grayL);
        cv::imshow("grayR",grayR);
        cv::waitKey(5);
    }
    log_i("Found good chessboard frms:"+to_string(j));

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
bool StereoCalib::calc_stereo()
{
    auto& objpoints = data_.objpoints;
    auto& imgpointsL = data_.imgpointsL;
    auto& imgpointsR = data_.imgpointsR;

    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;
    
    auto& c0 = data_.cams[0];
    auto& c1 = data_.cams[1];

    // This step is performed to transformation between the two cameras and calculate Essential and 
    // Fundamenatl matrix
    cv::stereoCalibrate(objpoints,
                        imgpointsL,
                        imgpointsR,
                        c0.Knew,
                        c0.dist,
                        c1.Knew,
                        c1.dist,
                        data_.sz_img,
                        data_.Rot,
                        data_.Trns,
                        data_.Emat,
                        data_.Fmat,
                        flag,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    
    
    return true;
}


//------
bool StereoCalib::calc_rectify()
{
    auto& sz = data_.sz_img;
    //----
    // Once we know the transformation between the two cameras we can perform 
    // stereo rectification
    auto& c0 = data_.cams[0];
    auto& c1 = data_.cams[1];    
    //----
    // Once we know the transformation between the two cameras we can perform 
    // stereo rectification
    cv::stereoRectify(c0.Knew,
                    c0.dist,
                    c1.Knew,
                    c1.dist,
                    sz,
                    data_.Rot,
                    data_.Trns,
                    c0.M_rect,
                    c1.M_rect,
                    c0.M_proj,
                    c1.M_proj,
                    data_.Q,
                    1);
    //----
    c0.calc_rectify(sz);
    c1.calc_rectify(sz);
    return true;
}
//----
void StereoCalib::CamSC::calc_rectify(const cv::Size& sz_img)
{

    cv::initUndistortRectifyMap(Knew,
                                dist,
                                M_rect,
                                M_proj,
                                sz_img,
                                CV_16SC2,
                                mapx,
                                mapy);
    
       
}
//-----
bool StereoCalib::Data::save(const string& sf)
{
    cv::FileStorage f = cv::FileStorage(sf, cv::FileStorage::WRITE);
    if(!f.isOpened())
    {
        log_e("Failed open file:"+sf);
        return false;
    }
    //----
    auto& c0 = cams[0];
    auto& c1 = cams[1];    

    f.write("L_mapx",c0.mapx);
    f.write("L_mapy",c0.mapy);
    f.write("R_mapx",c1.mapx);
    f.write("R_mapy",c1.mapy);

    log_i("Stereo calib data saved to:"+sf);

    return true;
}

//-----
cv::Mat StereoCalib::CamSC::remap(cv::Mat im)
{
    cv::Mat imr;
    cv::remap(im,
            imr,
            mapx,
            mapy,
            cv::INTER_LANCZOS4,
            cv::BORDER_CONSTANT,
            0);
     return imr;

}
