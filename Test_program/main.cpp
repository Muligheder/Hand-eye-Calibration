// include the librealsense C++ header file
/* REAL SENSE CAPTURE EXAMPLE
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    //Get each frame
    rs2::frame color_frame = frames.get_color_frame();

    // Creating OpenCV Matrix from a color image
    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    // Display in a GUI
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", color);

    waitKey(0);
    return 0;
}*/




// CHESSBOARD EXAMPLE
/*
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/aruco/charuco.hpp>
#include <stdio.h>
#include <iostream>

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9};

int main()
{
  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;

  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpoints;

  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }


  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;
  // Path of the folder containing checkerboard images
  std::string path = "/home/anders/opencv_test/Charuco.png";

  cv::glob(path, images);

  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checker board corners
  std::vector<cv::Point2f> corner_pts;
  bool success;

  // Looping over all the images in the directory
  for(int i{0}; i<images.size(); i++)
  {
    frame = cv::imread(images[i]);
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true
    success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH |  cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    /*
     * If desired number of corner are detected,
     * we refine the pixel coordinates and display
     * them on the images of checker board
    */
 /*
    if(success)
    {
      cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
    }

    cv::imshow("Image",frame);
    cv::waitKey(0);
  }

  cv::destroyAllWindows();

  cv::Mat cameraMatrix,distCoeffs,R,T;

  /*
   * Performing camera calibration by
   * passing the value of known 3D points (objpoints)
   * and corresponding pixel coordinates of the
   * detected corners (imgpoints)
  */
/*
  cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

  std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
  std::cout << "distCoeffs : " << distCoeffs << std::endl;
  std::cout << "Rotation vector : " << R << std::endl;
  std::cout << "Translation vector : " << T << std::endl;

  return 0;
}
*/

// PCL EXAMPLE WITH REALSENSE
/*#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "/home/anders/librealsense/examples/example.hpp" // Include short list of convenience functions for rendering

#include <iostream>



using namespace std;

#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;

using namespace cv;


// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}


ptr_cloud points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", colorr);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    ptr_cloud cloud(new point_cloud);

    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);

    }

   return cloud;
}






int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    while(app) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points);

        std::cout <<"I did it" <<std::endl;
        ptr_cloud cloud = points_to_pcl(points, color);
        pcl::io::savePCDFileASCII("cloud_test.pcd", *cloud);


            waitKey(0);


    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}*/

// CHARUCO BOARD EXAMPLE
/*
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
    cv::Mat img = cv::imread("/home/anders/Test_program/build/BoardImage.jpg",cv::IMREAD_COLOR);
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
        break;
    case 3:
        detectCharucoBoardWithCalibrationPose();
        break;
    default:
        break;
    }
    return 0;
}*/

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
/*
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

namespace {
const char* about =
        "Calibration using a ChArUco board\n"
        "  To capture a frame for calibration, press 'c',\n"
        "  If input comes from video, press any key for next frame\n"
        "  To finish capturing, press 'ESC' key and calibration starts.\n";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{@outfile |<none> | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       | false | Apply refind strategy }"
        "{zt       | false | Assume zero tangential distortion }"
        "{a        |       | Fix aspect ratio (fx/fy) to this value }"
        "{pc       | false | Fix the principal point at the center }"
        "{sc       | false | Show detected chessboard corners after calibration }";
}

/**
 */
/*
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}



/**
 */
/*
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}



/**
 */
/*
int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl");
    float markerLength = parser.get<float>("ml");
    int dictionaryId = parser.get<int>("d");
    string outputFile = parser.get<string>(0);

    bool showChessboardCorners = parser.get<bool>("sc");

    int calibrationFlags = 0;
    float aspectRatio = 1;
    if(parser.has("a")) {
        calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = parser.get<float>("a");
    }
    if(parser.get<bool>("zt")) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    if(parser.get<bool>("pc")) calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    bool refindStrategy = parser.get<bool>("rs");
    int camId = parser.get<int>("ci");
    String video;

    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    // collect data from each frame
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    Size imgSize;

    //while(inputVideo.grab())
   // while(true)
   // {
        Mat image, imageCopy;
        vector<cv::String> fn;
        glob("/home/anders/Test_program/build/images/*.jpg", fn, false);

        vector<Mat> images;
        size_t count = fn.size(); //number of png files in images folder
        for (size_t i=0; i<count; i++)
        {
            images.push_back(imread(fn[i]));

       // cv::Mat img = cv::imread("/home/anders/Test_program/build/BoardImage.jpg",cv::IMREAD_COLOR);
        inputVideo.retrieve(image);

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        cout << images.size()<< "<-- what"<<endl;
        // detect markers
        aruco::detectMarkers(images[i], dictionary, corners, ids, detectorParams, rejected);

        cout << ids.size()<< "<-- what1"<<endl;
        // refind strategy to detect more markers
        if(refindStrategy) aruco::refineDetectedMarkers(images[i], board, corners, ids, rejected);

        // interpolate charuco corners
        Mat currentCharucoCorners, currentCharucoIds;
        if(ids.size() > 0)
            aruco::interpolateCornersCharuco(corners, ids, images[i], charucoboard, currentCharucoCorners,
                                             currentCharucoIds);

        // draw results
        images[i].copyTo(imageCopy);
        if(ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners);

        if(currentCharucoCorners.total() > 0)
            aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
        //cout << "Escape" << endl;
        //if(key == 'c' && ids.size() > 0)
        //{
            cout << "Frame captured" << endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(images[i]);
            imgSize = images[i].size();
        //}

        }
   // }
    cout << allCorners.size() << "<-- number of corners"<<endl;
    cout << allIds.size() << "<-- number of ids"<<endl;
    cout << allImgs.size() << "<-- number of images"<<endl;
    if(allIds.size() < 1)
    {
        cerr << "Not enough captures for calibration" << endl;
        return 0;
    }
    else
        cout << "Starting prep for calibration" << endl;

    Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO)
    {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++)
    {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++)
        {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    cout << "Calibrate camera using aruco markers..."<<endl;
    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
                                              distCoeffs, noArray(), noArray(), calibrationFlags);

    cout << "Done"<<endl;


    cout << "Prepare data for charuco calibration..."<<endl;
    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    vector< Mat > allCharucoCorners;
    vector< Mat > allCharucoIds;
    vector< Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        Mat currentCharucoCorners, currentCharucoIds;
        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if(allCharucoCorners.size() < 4)
    {
        cerr << "Not enough corners for calibration" << endl;
        return 0;
    }
    cout << "Preparation done"<<endl;
    cout << "Calibrating using charuco..."<<endl;

    // calibrate camera using charuco
    repError =
        aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);
    cout << allCharucoCorners.size() << "<-- number of corners"<<endl;
    cout << allCharucoIds.size() << "<-- number of ids"<<endl;
    //cout << allImgs.size() << "<-- number of images"<<endl;
    cout << "Rep Error: " << repError << endl;
    cout << "Done"<<endl;
    bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
                                    cameraMatrix, distCoeffs, repError);
    if(!saveOk)
    {
        cerr << "Cannot save output file" << endl;
        return 0;
    }

    cout << "Rep Error: " << repError << endl;
    cout << "Rep Error Aruco: " << arucoRepErr << endl;
    cout << "Calibration saved to " << outputFile << endl;
    cout <<"./Test_program ""calib.txt"" -w=5 -h=7 -l=100 -s=10 -d=10 -sl=0.04 -ml=0.02 <=== start parameter to run the test in terminal"<<endl;
    // show interpolated charuco corners for debugging
    if(showChessboardCorners) {
        for(unsigned int frame = 0; frame < filteredImages.size(); frame++)
        {
            Mat imageCopy = filteredImages[frame].clone();
            if(allIds[frame].size() > 0)
            {

                if(allCharucoCorners[frame].total() > 0)
                {
                    aruco::drawDetectedCornersCharuco( imageCopy, allCharucoCorners[frame],
                                                       allCharucoIds[frame]);
                }
            }

            imshow("out", imageCopy);
            char key = (char)waitKey(0);
            if(key == 27) break;
        }
    }

    return 0;
}*/


// My_handeye.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
//#include "pch.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/aruco/charuco.hpp>


using namespace cv;
using namespace std;

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
Vec3f rotationMatrixToEulerAngles(Mat &R);
float rad2deg(float radian);
float deg2rad(float degree);
Mat ReverseVector(Mat &v);

int main()
{
    //TEST CODE FOR CORNER DETECTION
    /*
    Mat img = imread("/home/anders/Hand-eye-Calibration/Test_program/build/poses_and_images/images/cal_01.bmp",33);
        imshow("image",img);
        moveWindow("image",40,40);

        Size patternsize1(7,5); //interior number of corners
        Mat gray;
        cvtColor(img,gray,COLOR_BGR2GRAY);//source image
        vector<Point2f> corners; //this will be filled by the detected corners

        //CALIB_CB_FAST_CHECK saves a lot of time on images
        //that do not contain any chessboard corners
        bool patternfound1 = findChessboardCorners(gray, patternsize1, corners,
                            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                            + CALIB_CB_FAST_CHECK);

        if(patternfound1)
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        drawChessboardCorners(img, patternsize1, Mat(corners), patternfound1);

        imshow("result",img);
        moveWindow("result",img.cols/2,100);
        waitKey(0);
    */

    // Camera calibration information

    std::vector<double> distortionCoefficients(5);  // camera distortion
    distortionCoefficients[0] = 9.6349551984637724e-02;
    distortionCoefficients[1] = -3.3260675111130217e-01;
    distortionCoefficients[2] = 0;
    distortionCoefficients[3] = 0;
    distortionCoefficients[4] = 2.5833277679122602e-01;

    double f_x = 1.2993539019658076e+03; // Focal length in x axis
    double f_y = 1.2993539019658076e+03; // Focal length in y axis (usually the same?)
    double c_x = 960; // Camera primary point x
    double c_y = 540; // Camera primary point y

    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cameraMatrix.at<float>(0, 0) = f_x;
    cameraMatrix.at<float>(0, 1) = 0.0;
    cameraMatrix.at<float>(0, 2) = c_x;
    cameraMatrix.at<float>(1, 0) = 0.0;
    cameraMatrix.at<float>(1, 1) = f_y;
    cameraMatrix.at<float>(1, 2) = c_y;
    cameraMatrix.at<float>(2, 0) = 0.0;
    cameraMatrix.at<float>(2, 1) = 0.0;
    cameraMatrix.at<float>(2, 2) = 1.0;

    Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);
    //

    std::vector<Mat> R_gripper2base;
    std::vector<Mat> t_gripper2base;
    std::vector<Mat> R_target2cam;
    std::vector<Mat> t_target2cam;
    Mat R_cam2gripper = (Mat_<float>(3, 3));
    Mat t_cam2gripper = (Mat_<float>(3, 1));

    vector<String> fn;
    glob("/home/anders/Hand-eye-Calibration/Test_program/build/poses_and_images/images/*.bmp", fn, false);

    vector<Mat> images;
    size_t num_images = fn.size(); //number of bmp files in images folder
    cout << "number of images"<< num_images<<endl;
    Size patternsize(5, 7); //number of centers
    vector<Point2f> centers; //this will be filled by the detected centers
    float cell_size = 30;
    vector<Point3f> obj_points;

    R_gripper2base.reserve(num_images);
    t_gripper2base.reserve(num_images);
    R_target2cam.reserve(num_images);
    t_target2cam.reserve(num_images);

    for (int i = 0; i < patternsize.height; ++i)
        for (int j = 0; j < patternsize.width; ++j)
            obj_points.push_back(Point3f(float(j*cell_size),
                                         float(i*cell_size), 0.f));
cout <<"Objectpoints: " <<endl<<obj_points<<endl;
cout <<"Objectpoints: " <<endl<<obj_points.size()<<endl;
    for (size_t i = 0; i < num_images; i++)
        images.push_back(imread(fn[i]));


    Mat frame;

    for (size_t i = 0; i < num_images; i++)
    {
        frame = imread(fn[i]); //source image
        Mat gray;
        cvtColor(frame,gray,COLOR_BGR2GRAY);

        //  imshow("window",gray);
        // waitKey(0);

        bool patternfound = findChessboardCorners(frame, patternsize, centers);
        if (patternfound)
        {
          //  cornerSubPix(gray, centers, Size(1, 1), Size(-1, -1),
            //                  TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            drawChessboardCorners(frame, patternsize, Mat(centers), patternfound);
            // imshow("window", frame);
           //int key = cv::waitKey(0) & 0xff;
            solvePnP(Mat(obj_points), Mat(centers), cameraMatrix, distortionCoefficients, rvec, tvec,false,SOLVEPNP_ITERATIVE);
           // cout <<"Rotation vetor = "<<endl<<" " <<rvec<<endl<<endl;


            Mat R;
            Rodrigues(rvec, R); // R is 3x3
            R_target2cam.push_back(R);
            t_target2cam.push_back(tvec);
            Mat T = Mat::eye(4, 4, R.type()); // T is 4x4
            T(Range(0, 3), Range(0, 3)) = R * 1; // copies R into T
            T(Range(0, 3), Range(3, 4)) = tvec * 1; // copies tvec into T

            cout << "T = " << endl << " " << T << endl << endl;

        }
        cout << patternfound << endl;
    }

    //EULER=ZYX
    //    Vec3f theta_01{ deg2rad(-153.61), deg2rad(8.3),   deg2rad(-91.91) };
    //    Vec3f theta_02{ deg2rad(-166.71), deg2rad(3.04),  deg2rad(-93.31) };
    //    Vec3f theta_03{ deg2rad(-170.04), deg2rad(24.92), deg2rad(-88.29) };
    //    Vec3f theta_04{ deg2rad(-165.71), deg2rad(24.68), deg2rad(-84.85) };
    //    Vec3f theta_05{ deg2rad(-160.18), deg2rad(-15.94),deg2rad(-56.24) };
    //    Vec3f theta_06{ deg2rad(175.68),  deg2rad(10.95), deg2rad(180) };
    //    Vec3f theta_07{ deg2rad(175.73),  deg2rad(45.78), deg2rad(-179.92) };
    //    Vec3f theta_08{ deg2rad(-165.34), deg2rad(47.37), deg2rad(-166.25) };
    //    Vec3f theta_09{ deg2rad(-165.62), deg2rad(17.95), deg2rad(-166.17) };
    //    Vec3f theta_10{ deg2rad(-151.99), deg2rad(-14.59),deg2rad(-94.19) };

    //Part2
    Vec3f theta_01{ deg2rad(-37.2789), deg2rad(89.5008),   deg2rad(144.7559) };
    Vec3f theta_02{ deg2rad(-37.2546), deg2rad(89.5016),  deg2rad(144.7818) };
    Vec3f theta_03{ deg2rad(-37.5546), deg2rad(89.5013), deg2rad(144.4824) };
    Vec3f theta_04{ deg2rad(-37.4683), deg2rad(89.5022), deg2rad(144.5671) };
    Vec3f theta_05{ deg2rad(71.1106), deg2rad(89.2108),deg2rad(-106.8568) };
    Vec3f theta_06{ deg2rad(169.2179),  deg2rad(71.3478), deg2rad(-3.5778) };
    Vec3f theta_07{ deg2rad(-11.4956),  deg2rad(65.2075), deg2rad(179.6673) };
    Vec3f theta_08{ deg2rad(-52.3002), deg2rad(58.4651), deg2rad(143.2993) };
    Vec3f theta_09{ deg2rad(48.8168), deg2rad(47.3050), deg2rad(-128.2089) };
    Vec3f theta_10{ deg2rad(24.0773), deg2rad(60.5126),deg2rad(-148.4420) };



    Mat robot_rot_01 = eulerAnglesToRotationMatrix(theta_01);
    Mat robot_rot_02 = eulerAnglesToRotationMatrix(theta_02);
    Mat robot_rot_03 = eulerAnglesToRotationMatrix(theta_03);
    Mat robot_rot_04 = eulerAnglesToRotationMatrix(theta_04);
    Mat robot_rot_05 = eulerAnglesToRotationMatrix(theta_05);
    Mat robot_rot_06 = eulerAnglesToRotationMatrix(theta_06);
    Mat robot_rot_07 = eulerAnglesToRotationMatrix(theta_07);
    Mat robot_rot_08 = eulerAnglesToRotationMatrix(theta_08);
    Mat robot_rot_09 = eulerAnglesToRotationMatrix(theta_09);
    Mat robot_rot_10 = eulerAnglesToRotationMatrix(theta_10);

    //    const Mat robot_tr_01 = (Mat_<float>(3, 1) << 781.2, 338.59, 903.48);
    //    const Mat robot_tr_02 = (Mat_<float>(3, 1) << 867.65, 382.52, 884.42);
    //    const Mat robot_tr_03 = (Mat_<float>(3, 1) << 856.91, 172.99, 964.61);
    //    const Mat robot_tr_04 = (Mat_<float>(3, 1) << 748.81, 146.75, 1043.29);
    //    const Mat robot_tr_05 = (Mat_<float>(3, 1) << 627.66, 554.08, 920.85);
    //    const Mat robot_tr_06 = (Mat_<float>(3, 1) << 715.06, 195.96, 889.38);
    //    const Mat robot_tr_07 = (Mat_<float>(3, 1) << 790.9, 196.29, 1117.38);
    //    const Mat robot_tr_08 = (Mat_<float>(3, 1) << 743.5, 283.93, 1131.92);
    //    const Mat robot_tr_09 = (Mat_<float>(3, 1) << 748.9, 288.19, 910.58);
    //    const Mat robot_tr_10 = (Mat_<float>(3, 1) << 813.18, 400.44, 917.16);

    //Part 2
     Mat robot_tr_01 = (Mat_<float>(3, 1) << -941.7910, 8.6871, 120.6610);
     Mat robot_tr_02 = (Mat_<float>(3, 1) << -947.4020, -153.0410, 101.1080);
     Mat robot_tr_03 = (Mat_<float>(3, 1) << -950.4640, -150.7550, 537.3240);
     Mat robot_tr_04 = (Mat_<float>(3, 1) << -944.2820, 22.7132, 536.4470);
     Mat robot_tr_05 = (Mat_<float>(3, 1) << -1197.3, 69.3, 335.5);
     Mat robot_tr_06 = (Mat_<float>(3, 1) << -1076.7, -104.6, 323.4);
     Mat robot_tr_07 = (Mat_<float>(3, 1) << -1162.7, -110.8, 396.7);
     Mat robot_tr_08 = (Mat_<float>(3, 1) << -1073.8, -128.3, 438.5);
     Mat robot_tr_09 = (Mat_<float>(3, 1) << -1073.8, -128.3, 438.5);
     Mat robot_tr_10 = (Mat_<float>(3, 1) << -1171.5, -166.3, 630.0);

//     //Reverse the translation vector from XYZ to ZYX
//     robot_tr_01= ReverseVector(robot_tr_01);
//     robot_tr_02= ReverseVector(robot_tr_02);
//     robot_tr_03= ReverseVector(robot_tr_03);
//     robot_tr_04= ReverseVector(robot_tr_04);
//     robot_tr_05= ReverseVector(robot_tr_05);
//     robot_tr_06= ReverseVector(robot_tr_06);
//     robot_tr_07= ReverseVector(robot_tr_07);
//     robot_tr_08= ReverseVector(robot_tr_08);
//     robot_tr_09= ReverseVector(robot_tr_09);
//     robot_tr_10= ReverseVector(robot_tr_10);


//     cout << "Robot_tr_011 = " << endl << " " << ReverseVector(robot_tr_01) << endl << endl;



    R_gripper2base.push_back(robot_rot_01);
    R_gripper2base.push_back(robot_rot_02);
    R_gripper2base.push_back(robot_rot_03);
    R_gripper2base.push_back(robot_rot_04);
    R_gripper2base.push_back(robot_rot_05);
    R_gripper2base.push_back(robot_rot_06);
    R_gripper2base.push_back(robot_rot_07);
    R_gripper2base.push_back(robot_rot_08);
    R_gripper2base.push_back(robot_rot_09);
    R_gripper2base.push_back(robot_rot_10);



    t_gripper2base.push_back(robot_tr_01);
    t_gripper2base.push_back(robot_tr_02);
    t_gripper2base.push_back(robot_tr_03);
    t_gripper2base.push_back(robot_tr_04);
    t_gripper2base.push_back(robot_tr_05);
    t_gripper2base.push_back(robot_tr_06);
    t_gripper2base.push_back(robot_tr_07);
    t_gripper2base.push_back(robot_tr_08);
    t_gripper2base.push_back(robot_tr_09);
    t_gripper2base.push_back(robot_tr_10);

    cout << "R_gripper2base = " << endl << " " << R_gripper2base[0] << endl << endl;
    cout << "t_gripper2base = " << endl << " " << t_gripper2base[0] << endl << endl;



    cout << R_gripper2base.size() << " R_gripper2base"<<endl;
    cout << t_gripper2base.size() << " t_gripper2base"<<endl;
    cout << R_target2cam.size() << " R_target2cam"<<endl;
    cout << R_target2cam.size() << " t_target2cam"<<endl;
    cout << R_cam2gripper.size() << " R_cam2gripper"<<endl;
    cout << t_cam2gripper.size() << " t_cam2gripper"<<endl<<endl;



    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, CALIB_HAND_EYE_TSAI);

    Vec3f R_cam2gripper_r = rotationMatrixToEulerAngles(R_cam2gripper);

    cout << "R_cam2gripper = " << endl << " " << R_cam2gripper << endl << endl;
    cout << "R_cam2gripper_r = " << endl << " " << R_cam2gripper_r << endl << endl;
    cout << "t_cam2gripper = " << endl << " " << t_cam2gripper << endl << endl;
}

Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(theta[2]), -sin(theta[2]),
        0, sin(theta[2]), cos(theta[2])
        );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3, 3) <<
        cos(theta[1]), 0, sin(theta[1]),
        0, 1, 0,
        -sin(theta[1]), 0, cos(theta[1])
        );

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3, 3) <<
        cos(theta[0]), -sin(theta[0]), 0,
        sin(theta[0]), cos(theta[0]), 0,
        0, 0, 1);

    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;

    return R;

}

float rad2deg(float radian) {
    double pi = 3.14159;
    return(radian * (180 / pi));
}

float deg2rad(float degree) {
    double pi = 3.14159;
    return(degree * (pi / 180));
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles.
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Vec3f(rad2deg(x), rad2deg(y), rad2deg(z));


}

Mat ReverseVector(Mat &v)
{

    Mat RV = Mat_<float>(3,1);

    RV.at<float>(0,0)=v.at<float>(0,2);
    RV.at<float>(0,1)=v.at<float>(0,1);
    RV.at<float>(0,2)=v.at<float>(0,0);

    return RV;
}
