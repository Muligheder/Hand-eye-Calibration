//// include the librealsense C++ header file
// //REAL SENSE CAPTURE EXAMPLE
//#include <librealsense2/rs.hpp>

//// include OpenCV header file
//#include <opencv2/opencv.hpp>

//using namespace std;
//using namespace cv;

//int main()
//{
//    //Contruct a pipeline which abstracts the device
//    rs2::pipeline pipe;

//    //Create a configuration for configuring the pipeline with a non default profile
//    rs2::config cfg;

//    //Add desired streams to configuration
//    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

//    //Instruct pipeline to start streaming with the requested configuration
//    pipe.start(cfg);

//    // Camera warmup - dropping several first frames to let auto-exposure stabilize
//    rs2::frameset frames;
//    for(int i = 0; i < 30; i++)
//    {
//        //Wait for all configured streams to produce a frame
//        frames = pipe.wait_for_frames();
//    }

//    //Get each frame
//    rs2::frame color_frame = frames.get_color_frame();

//    // Creating OpenCV Matrix from a color image
//    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

//    // Display in a GUI
//    namedWindow("Display Image", WINDOW_AUTOSIZE );
//    imshow("Display Image", color);

//    waitKey(0);
//    return 0;
//}

// CHARUCO BOARD EXAMPLE

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>


#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include <fstream>
#include <ctime>
#include <cstdio>


namespace {
const char* about = "A tutorial code on charuco board creation and detection of charuco board with and without camera caliberation";
const char* keys = "{c        |       | Put value of c=1 to create charuco board;\nc=2 to detect charuco board without camera calibration;\nc=3 to detect charuco board with camera calibration and Pose Estimation}";
}
void createBoard();
void detectCharucoBoardWithCalibrationPose();
void detectCharucoBoardWithoutCalibration();
static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(600, 500), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}
void detectCharucoBoardWithCalibrationPose()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else {

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        while (inputVideo.grab()) {
            cv::Mat image;
            cv::Mat imageCopy;
            inputVideo.retrieve(image);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
            // if at least one marker detected
            if (markerIds.size() > 0)
            {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0)
                {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    cv::Vec3d rvec, tvec;
                    // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                }
            }
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
    }
}

void detectCharucoBoardWithoutCalibration()
{
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat img = cv::imread("/home/anders/Master/Hand-eye-Calibration/Test_program/build/BoardImage.jpg",cv::IMREAD_COLOR);
    std::cout << "image read complete" << std::endl;
    //cv::VideoCapture inputVideo;
    //inputVideo.open(0);
    std::string filename = "calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else{
        std::cout << "camera file loaded" << std::endl;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    //while (inputVideo.grab()) {

        cv::Mat image, imageCopy;
        //inputVideo.retrieve(image);
        img.copyTo(imageCopy);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(img, board->dictionary, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds);
            // if at least one charuco corner detected
            if (charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));

                std::vector<cv::Mat> rvesc,tvecs;
                std::vector<cv::Point3f> stdDev,stdEx,pvr;
              //  std::cout << rvesc << std::endl;

                if (markerIds.size() >0 && charucoIds.size() >0)
                    cv::aruco::calibrateCameraCharuco(charucoCorners,charucoIds,board,cv::Size(img.rows,img.cols),cameraMatrix,distCoeffs,rvesc,tvecs,stdDev,stdEx,pvr,0);










        cv::imshow("out", imageCopy);
        std::cout <<"An image named Charuco.jpg is generated in folder containing this file"<< std::endl;
        std::cout << img.size() << std::endl;
      //  std::cout << "Rotation" << rvesc << std::endl;







        cv::imwrite("Charuco.jpg",imageCopy);
      //  char key = (char)cv::waitKey(30);
    //    if (key == 27)
          //  break;
    }
//}
}}
int main(int argc, char* argv[])
{
//    cv::CommandLineParser parser(argc, argv, keys);
//    parser.about(about);
//    if (argc < 2) {
//        parser.printMessage();
//        return 0;
//    }
//    int choose = parser.get<int>("c");
//    switch (choose) {
//    case 1:
//        createBoard();
//        std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
//        break;
//    case 2:
//        detectCharucoBoardWithoutCalibration();
//        break;
//    case 3:
//        detectCharucoBoardWithCalibrationPose();
//        break;
//    default:
//        break;
//    }
//    return 0;

    // { 0.617054, -0.0902524, 0.142154, -2.84451, -1.23569, -0.0885921 } first position
    // { 0.383671, -0.0848814, 0.139784, -2.84177, -1.23832, -0.0894176 } secound position
    std::ofstream myfile("error_test_robot.txt", std::ios::app | std::ofstream::binary);

    std::vector<std::vector<double>> v;

    std::ifstream in( "/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/Robot_poses.txt" );
    std::string record;

    while ( std::getline( in, record ) )
    {
        std::istringstream is( record );
        std::vector<double> row( ( std::istream_iterator<double>( is ) ),
                                 std::istream_iterator<double>() );
        v.push_back( row );

    }


         double paper_size = 0.23;
        cv::Vec3d a{v[0][0], v[0][1], v[0][2]};
        cv::Vec3d b{v[1][0], v[1][1], v[1][2]};

        std::cout << "The distance between point a and b: " << cv::norm(a,b,cv::NORM_L2) << std::endl;

        double c=abs(/*paper_size-*/cv::norm(a,b,cv::NORM_L2));
        std::cout << "The error is: " << c << std::endl;

        myfile << c << std::endl;
    std::ofstream myfile1;
    myfile1.open("/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/Robot_poses.txt", std::ofstream::out | std::ofstream::trunc);









}

/*
//! [charucohdr]
#include <opencv2/aruco/charuco.hpp>
//! [charucohdr]
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

namespace {
const char* about = "A tutorial code on charuco board creation and detection of charuco board with and without camera caliberation";
const char* keys = "{c        |       | Put value of c=1 to create charuco board;\nc=2 to detect charuco board without camera calibration;\nc=3 to detect charuco board with camera calibration and Pose Estimation}";
}

void createBoard();
void detectCharucoBoardWithCalibrationPose();
void detectCharucoBoardWithoutCalibration();

static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    //! [createBoard]
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(600, 500), boardImage, 10, 1);
    //! [createBoard]
    cv::imwrite("BoardImage.jpg", boardImage);
}

//! [detwcp]
void detectCharucoBoardWithCalibrationPose()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    //! [matdiscoff]
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "/home/anders/Test_program/build/calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    //! [matdiscoff]
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else {
        //! [dictboard]
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        //! [dictboard]
        // while (inputVideo.grab())
        while (true)
        {
            //! [inputImg]
            cv::Mat image;
            //! [inputImg]
            cv::Mat imageCopy;
            //inputVideo.retrieve(image);
            cv::Mat img = cv::imread("/home/anders/Test_program/build/BoardImage.jpg",cv::IMREAD_COLOR);
            img.copyTo(imageCopy);
            //! [midcornerdet]
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(img, board->dictionary, markerCorners, markerIds, params);
            //! [midcornerdet]
            // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                //! [charidcor]
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                //! [charidcor]
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    //! [detcor]
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    //! [detcor]
                    cv::Vec3d rvec, tvec;
                    //! [pose]
                    // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    //! [pose]
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                }
            }
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
    }
//}
//! [detwcp]

//! [detwc]
void detectCharucoBoardWithoutCalibration()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);

    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
  //  while (inputVideo.grab())
    while(true)
    {
        cv::Mat image, imageCopy;
        //inputVideo.retrieve(image);
        cv::Mat img = cv::imread("/home/anders/Test_program/build/BoardImage.jpg",cv::IMREAD_COLOR);
        img.copyTo(imageCopy);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(img, board->dictionary, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            //! [charidcorwc]
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds);
            //! [charidcorwc]
            // if at least one charuco corner detected
            if (charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
        }

        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;

    }
}
//! [detwc]

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (argc < 2) {
        parser.printMessage();
        return 0;
    }
    int choose = parser.get<int>("c");
    switch (choose) {
    case 1:
        createBoard();
        std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
        break;
    case 2:
        detectCharucoBoardWithoutCalibration();
        std::cout << "An otu" << std::endl;
        break;
    case 3:
        detectCharucoBoardWithCalibrationPose();
        break;
    default:
        break;
    }
    return 0;
}*/
//int main()
//{
//    return 0;
//}
