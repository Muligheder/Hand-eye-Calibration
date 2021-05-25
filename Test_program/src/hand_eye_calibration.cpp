#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include <fstream>
#include <ctime>
#include <cstdio>
//#include "pch.h"

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

#include <visp3/vision/vpHandEyeCalibration.h>

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

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <numeric>

using namespace cv;
using namespace std;
#define DEBUG_LEVEL2 0
Mat eulerAnglesToRotationMatrix(Vec3f &theta);
Vec3f rotationMatrixToEulerAngles(Mat &R);
float rad2deg(float radian);
float deg2rad(float degree);
Mat ReverseVector(Mat &v);
void print(std::vector<double> const &input);
Vec3f Rad2degV(Vec3f v);
Vec3f Poly_Scale(Vec3f theta_rv);
Mat TransposeVector(Vec3d &v);
std::vector<double> Pose_inv(std::vector<double> v);
void errorEstimate(const std::vector<Mat> &R_gripper2base, const std::vector<Mat> &t_gripper2base, const std::vector<Mat> &R_target2cam, const std::vector<Mat> &t_target2cam, const Mat &R_cam2gripper, const Mat &t_cam2gripper);
int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou);
void calibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe, const vpHomogeneousMatrix &eMc);
int CHECKERBOARD[2]{5,8};

int main()
{
//****************************************************** CAMERA SETUP ************************************************************************************'//

     //Camera calibration information from API 1920x1080
    std::vector<double> distortionCoefficients(5);  // camera distortion
    distortionCoefficients[0] = 1.83375015854836e-01;
    distortionCoefficients[1] = -5.65327823162079e-01;
    distortionCoefficients[2] = -1.45305093610659e-04;
    distortionCoefficients[3] = 5.0213176291436e-04;
    distortionCoefficients[4] = 5.08288383483887e-01;

    float f_x = 1364.6708984375; // Focal length in x axis
    float f_y = 1364.94384765625; // Focal length in y axis
    float c_x = 967.542907714844; // Camera primary point x
    float c_y = 540.435607910156; // Camera primary point y

////    // Camera calibration information from calibration 1920x1080
//    std::vector<double> distortionCoefficients(5);  // camera distortion
//    distortionCoefficients[0] = 0.152762;
//    distortionCoefficients[1] = -0.408405;
//    distortionCoefficients[2] = 0.00366511;
//    distortionCoefficients[3] = 0.000347318;
//    distortionCoefficients[4] = 0.25188;

//    float f_x = 1373.745226037709; // Focal length in x axis
//    float f_y = 1372.453906056477; // Focal length in y axis
//    float c_x = 966.2194707820457; // Camera primary point x
//    float c_y = 553.6856848982575; // Camera primary point y

//    std::vector<double> distortionCoefficients(5);  // camera distortion
//    distortionCoefficients[0] = 0.0941717;
//    distortionCoefficients[1] = 0.0214873;
//    distortionCoefficients[2] = 0.00493083;
//    distortionCoefficients[3] = 0.000244589;
//    distortionCoefficients[4] = -0.746346;

//    float f_x = 1365.567566059209; // Focal length in x axis
//    float f_y = 1364.670083928078; // Focal length in y axis
//    float c_x = 964.7070499563438; // Camera primary point x
//    float c_y = 559.3336078254289; // Camera primary point y


//    // Camera calibration information from API   960x540
//    std::vector<double> distortionCoefficients(5);  // camera distortion
//    distortionCoefficients[0] = 1.83375015854836e-01;
//    distortionCoefficients[1] = -5.65327823162079e-01;
//    distortionCoefficients[2] = -1.45305093610659e-04;
//    distortionCoefficients[3] = 5.0213176291436e-04;
//    distortionCoefficients[4] = 5.08288383483887e-01;

//    float f_x = 6.8233544921875e+02; // Focal length in x axis
//    float f_y = 6.82471923828125e+02; // Focal length in y axis
//    float c_x = 483.771453857422; // Camera primary point x
//    float c_y = 270.217803955078; // Camera primary point y

//    //From calibration
//    std::vector<double> distortionCoefficients(5);  // camera distortion
//    distortionCoefficients[0] = 0.1895977955383683;
//    distortionCoefficients[1] = -0.4229510008378715;
//    distortionCoefficients[2] = 0.005908837920285038;
//    distortionCoefficients[3] = 0.002710859932538448;
//    distortionCoefficients[4] = 0.2606413790013677;
//    //0.1483631349257573, -0.1137807157391542, 0.007214638913007516, 0.003111347347643775, -0.4468245267930271
//    //706.4577138648071, 0, 486.8361380817768;    0, 706.5110947768565, 278.5669459125713;    0, 0, 1]

//    float f_x = 707.6644293843714; // Focal length in x axis
//    float f_y = 707.715907354843; // Focal length in y axis
//    float c_x = 486.5038794426711; // Camera primary point x
//    float c_y = 277.3043843816276; // Camera primary point y

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
   // cout << cameraMatrix << endl;

    cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);

    std::vector<cv::Mat> R_gripper2base;
    //std::vector<cv::Mat> R_gripper2base_Euler;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    cv::Mat R_cam2gripper_TSAI = (cv::Mat_<double>(3, 3));
    cv::Mat R_cam2gripper_HORAUD = (cv::Mat_<double>(3, 3));
    cv::Mat R_cam2gripper_PARK = (cv::Mat_<double>(3, 3));
    cv::Mat R_cam2gripper_ANDREFF = (cv::Mat_<double>(3, 3));
    cv::Mat R_cam2gripper_DAN = (cv::Mat_<double>(3, 3));
    cv::Mat t_cam2gripper_TSAI = (cv::Mat_<double>(3, 1));
    cv::Mat t_cam2gripper_HORAUD = (cv::Mat_<double>(3, 1));
    cv::Mat t_cam2gripper_PARK = (cv::Mat_<double>(3, 1));
    cv::Mat t_cam2gripper_ANDREFF = (cv::Mat_<double>(3, 1));
    cv::Mat t_cam2gripper_DAN = (cv::Mat_<double>(3, 1));

    // Euler
//    cv::Mat R_cam2gripper_Euler_TSAI = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_Euler_HORAUD = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_Euler_PARK = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_Euler_ANDREFF = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_Euler_DAN = (cv::Mat_<float>(3, 3));
//    cv::Mat t_cam2gripper_Euler_TSAI = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_Euler_HORAUD = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_Euler_PARK = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_Euler_ANDREFF = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_Euler_DAN = (cv::Mat_<float>(3, 1));

      // Creating vector to store vectors of 3D points for each checkerboard image
      std::vector<std::vector<cv::Point3f> > objpoints;

      // Creating vector to store vectors of 2D points for each checkerboard image
      std::vector<std::vector<cv::Point2f> > imgpoints;

      // Defining the world coordinates for 3D points
      std::vector<cv::Point3f> objp;

      // To calculate reprojection error
      std::vector<double> stdDeviationsIntrinsics,stdDeviationExtrinsics,perViewErrors;
      cv::Mat rvecs,tvecs;
      float Cell_size = 0.0286666666666;

      for(int i{0}; i<CHECKERBOARD[1]; i++)
      {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
          objp.push_back(cv::Point3f(float(j*Cell_size),float(i*Cell_size),0.f));
        //std::cout << objp << std::endl;

      }

      // Extracting path of individual image stored in a given directory
      std::vector<cv::String> images;
      // Path of the folder containing checkerboard images
      cv::String path = "/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/Images/*.bmp";

      cv::glob(path, images,false); // use natSort to get the correct image order or put a 0 before everything
      std::size_t num_images = images.size();
      cv::Mat frame, gray;
      // std::sort(images);

      R_gripper2base.reserve(num_images);
      t_gripper2base.reserve(num_images);
      R_target2cam.reserve(num_images);
      t_target2cam.reserve(num_images);

      // vector to store the pixel coordinates of detected checker board corners
      std::vector<cv::Point2f> corner_pts;
      bool success;
        int cool = 20;
      // Looping over all the images in the directory
      for(int i{0}; i<cool/*images.size()*/; i++)
      {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
        */
        if(success)
        {
          cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1);

          // refining pixel coordinates for given 2d points.
          cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

          // Displaying the detected corner points on the checker board
          cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

          objpoints.push_back(objp);
          imgpoints.push_back(corner_pts);

          float length = 0.05;
          int thickness = 3;

          // Finding transformation matrix
          cv::solvePnP(cv::Mat(objp), cv::Mat(corner_pts), cameraMatrix, distortionCoefficients, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);
          cv::drawFrameAxes(frame,cameraMatrix,distortionCoefficients,rvec,tvec,length,thickness);
          cv::Mat R;
                     cv::Rodrigues(rvec, R); // R is 3x3
//                     std::cout <<std::endl << "Tvec " << std::endl << tvec/1000 << std::endl;
//                     std::cout <<std::endl << "Rvec " << std::endl << rvec*1 << std::endl;
                     //R=R.t();// rotation of inverse
                     //tvec= -R * tvec; // translation of inverse
                     R_target2cam.push_back(R*1);
                     t_target2cam.push_back(tvec*1);

                     cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
                     T(cv::Range(0, 3), cv::Range(0, 3)) = R*1; // copies R into T
                     T(cv::Range(0, 3), cv::Range(3, 4)) = tvec*1; // copies tvec into T

//                     std::cout << "T = " << std::endl << " " << T << std::endl << std::endl;


        }

//        cv::imshow("Image",frame);
//        cv::imwrite("Axis.bmp",frame);
//        cv::waitKey(0);



      }
      // camera calibration which leads to reprojection error

      cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distortionCoefficients, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationExtrinsics, perViewErrors,0);
      std::ofstream myfile;
            myfile.open ("example.csv");
            for (int i = 0; i<perViewErrors.size(); i++)
            {
            myfile << perViewErrors[i];
            myfile << "\n";
            }
            myfile.close();

      float Avg_RMS = std::accumulate(perViewErrors.begin(), perViewErrors.end(), 0);
      for (auto& n : perViewErrors)
          Avg_RMS += n;

      /*
      tot_error=0
      total_points=0
      for i in xrange(len(obj_points)):
          reprojected_points, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
          reprojected_points=reprojected_points.reshape(-1,2)
          tot_error+=np.sum(np.abs(img_points[i]-reprojected_points)**2)
          total_points+=len(obj_points[i])

      mean_error=np.sqrt(tot_error/total_points)
      print "Mean reprojection error: ", mean_error*/

//      double error=0;
//      double tot_error=0;
//      double total_point=0;
//      std::vector<cv::Point2f> projected_pts;
//      for(int i=0; i<images.size(); i++)
//      {
//          cv::projectPoints(cv::Mat(objpoints[i]),rvecs[i],tvecs[i],cameraMatrix,distortionCoefficients,projected_pts);
//          error=cv::norm(cv::Mat(imgpoints[i]),cv::Mat(projected_pts),NORM_L2)/projected_pts.size();
//          tot_error+=error;
//      }
//      std::cout << "Reproejction error : " << std::endl <<" "<< tot_error/objpoints.size() << std::endl << std::endl;

      std::cout << "cameraMatrix : " << std::endl <<" "<< cameraMatrix << std::endl << std::endl;

      std::cout << "distCoeffs : ";
      print(distortionCoefficients);
      std::cout << std::endl << std::endl;
      std::cout << "Rotation vector : " << std::endl <<" " << rvecs << std::endl << std::endl;
      std::cout << "Translation vector : "<< std::endl <<" " << tvecs << std::endl << std::endl;

      std::cout << std::endl << "Size = "<< stdDeviationsIntrinsics.size() << " " << "standard deviation intrinsics = "  << std::endl << std::endl;
      print(stdDeviationsIntrinsics);
      std::cout<< std::endl << std::endl;

      std::cout << std::endl << "Size = "<< stdDeviationExtrinsics.size() << " "  << "standard deviation extrinsics = "  << std::endl << std::endl;
      print(stdDeviationExtrinsics);
      std::cout<< std::endl << std::endl;


      std::cout << std::endl << "Size = "<< perViewErrors.size() << std::endl << "RMS re-projection error estimated for each pattern view = " << std::endl;
      print(perViewErrors);
      std::cout<< std::endl << std::endl << "Avg_RMS = " << Avg_RMS/perViewErrors.size() << std::endl << std::endl;


// read data from file and seperate to vector of vector.
           std::vector<std::vector<double>> v;

           std::ifstream in( "/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/test.txt" );
           std::string record;

           while ( std::getline( in, record ) )
           {
               std::istringstream is( record );
               std::vector<double> row( ( std::istream_iterator<double>( is ) ),
                                        std::istream_iterator<double>() );
               v.push_back( row );

           }
           for(int i = 0; i < cool/*v.size()*/; i++)
           {
                cv::Mat theta_scaled, robot_rot, robot_tr;

                    // parsing roational data from robot //
                   Vec3d theta{ v[i][3], v[i][4], v[i][5]};
                   // scale vector to match ur interface
                   theta=Poly_Scale(theta);
                   // transpose vector
                   theta_scaled = TransposeVector(theta);
                   // from rotvector to rot matrix
                   cv::Rodrigues(theta_scaled,robot_rot);
                   // push back elements to vector of vectors I.e matrix
                   R_gripper2base.push_back(robot_rot);
                   //cout<< "Robot_rot: " << i << endl << R_gripper2base[i]<< endl << endl;

                   // Parsing translational data from robot //
                   robot_tr = (Mat_<double>(3, 1) << v[i][0],v[i][1],v[i][2]);
                   t_gripper2base.push_back(robot_tr);
                   //cout<< "Robot_tr: " << i << endl << t_gripper2base[i]<< endl << endl;
           }

    //**************************** RODRIGUES **************************************************
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_TSAI, t_cam2gripper_TSAI, CALIB_HAND_EYE_TSAI);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_HORAUD, t_cam2gripper_HORAUD, CALIB_HAND_EYE_HORAUD);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_ANDREFF, t_cam2gripper_ANDREFF, CALIB_HAND_EYE_ANDREFF);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_PARK, t_cam2gripper_PARK, CALIB_HAND_EYE_PARK);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_DAN, t_cam2gripper_DAN, CALIB_HAND_EYE_DANIILIDIS);
    Vec3f R_cam2gripper_rT = rotationMatrixToEulerAngles(R_cam2gripper_TSAI);
    Vec3f R_cam2gripper_rH = rotationMatrixToEulerAngles(R_cam2gripper_HORAUD);
    Vec3f R_cam2gripper_rA = rotationMatrixToEulerAngles(R_cam2gripper_ANDREFF);
    Vec3f R_cam2gripper_rP = rotationMatrixToEulerAngles(R_cam2gripper_PARK);
    Vec3f R_cam2gripper_rD = rotationMatrixToEulerAngles(R_cam2gripper_DAN);

    //calibrateHandEye();

    cv::Mat Rodrigues_Tsai, Rodrigues_Horaud, Rodrigues_Park, Rodrigues_Andreff, Rodrigues_Dan;
    cv::Rodrigues(R_cam2gripper_TSAI,Rodrigues_Tsai);
    cv::Rodrigues(R_cam2gripper_HORAUD,Rodrigues_Horaud);
    cv::Rodrigues(R_cam2gripper_PARK,Rodrigues_Park);
    cv::Rodrigues(R_cam2gripper_ANDREFF,Rodrigues_Andreff);
    cv::Rodrigues(R_cam2gripper_DAN,Rodrigues_Dan);


    // Rotation
    cout << "R_cam2gripper_TSAI = " << endl << " " << R_cam2gripper_TSAI << endl << endl;
    cout << "R_cam2gripper_rT = " << endl << " " << R_cam2gripper_rT << endl << endl;
    cout << " Rodrigues R_cam2gripper_rT = " << endl << " " << Rad2degV(Rodrigues_Tsai) << endl << endl;

    cout << "R_cam2gripper_HORAUD = " << endl << " " << R_cam2gripper_HORAUD << endl << endl;
    cout << "R_cam2gripper_rH = " << endl << " " << R_cam2gripper_rH << endl << endl;
    cout << " Rodrigues R_cam2gripper_rH = " << endl << " " << Rad2degV(Rodrigues_Horaud) << endl << endl;

    cout << "R_cam2gripper_PARK = " << endl << " " << R_cam2gripper_PARK << endl << endl;
    cout << "R_cam2gripper_rP = " << endl << " " << R_cam2gripper_rP << endl << endl;
    cout << " Rodrigues R_cam2gripper_rP = " << endl << " " << Rad2degV(Rodrigues_Park) << endl << endl;

    cout << "R_cam2gripper_ANDREFF = " << endl << " " << R_cam2gripper_ANDREFF << endl << endl;
    cout << "R_cam2gripper_rA = " << endl << " " << R_cam2gripper_rA << endl << endl;
    cout << " Rodrigues R_cam2gripper_rA = " << endl << " " << Rad2degV(Rodrigues_Andreff) << endl << endl;

    cout << "R_cam2gripper_DAN = " << endl << " " << R_cam2gripper_DAN << endl << endl;
    cout << "R_cam2gripper_rD = " << endl << " " << R_cam2gripper_rD << endl << endl;
    cout << " Rodrigues R_cam2gripper_rD = " << endl << " " << Rad2degV(Rodrigues_Dan) << endl << endl;

    // Translation
    cout << "t_cam2gripper_TSAI = " << endl << " " << t_cam2gripper_TSAI << endl << endl;
    cout << "t_cam2gripper_HORAUD = " << endl << " " << t_cam2gripper_HORAUD << endl << endl;
    cout << "t_cam2gripper_PARK = " << endl << " " << t_cam2gripper_PARK << endl << endl;
    cout << "t_cam2gripper_ANDREFF = " << endl << " " << t_cam2gripper_ANDREFF << endl << endl;
    cout << "t_cam2gripper_DAN = " << endl << " " << t_cam2gripper_DAN << endl << endl;

//    std::ofstream myfile_tsai;
//    myfile_tsai.open ("translation_Tsai.csv",std::ios::app);
//    // for (int i = 0; i<perViewErrors.size(); i++)
//    //{
//    myfile_tsai << t_cam2gripper_TSAI.t();
//    myfile_tsai << Rad2degV(Rodrigues_Tsai).t();
//    myfile_tsai << "\n";
//    //}
//    myfile_tsai.close();

//    std::ofstream myfile_Horaud;
//    myfile_Horaud.open ("translation_Horaud.csv",std::ios::app);
//    // for (int i = 0; i<perViewErrors.size(); i++)
//    //{
//    myfile_Horaud << t_cam2gripper_HORAUD.t();
//    myfile_Horaud << Rad2degV(Rodrigues_Horaud).t();
//    myfile_Horaud << "\n";
//    //}
//    myfile_Horaud.close();

//    std::ofstream myfile_park;
//    myfile_park.open ("translation_Park.csv",std::ios::app);
//    // for (int i = 0; i<perViewErrors.size(); i++)
//    //{
//    myfile_park << t_cam2gripper_PARK.t();
//    myfile_park << Rad2degV(Rodrigues_Park).t();
//    myfile_park << "\n";
//    //}
//    myfile_park.close();

//    std::ofstream myfile_Andreff;
//    myfile_Andreff.open ("translation_Andreff.csv",std::ios::app);
//    // for (int i = 0; i<perViewErrors.size(); i++)
//    //{
//    myfile_Andreff << t_cam2gripper_ANDREFF.t();
//    myfile_Andreff << Rad2degV(Rodrigues_Andreff).t();
//    myfile_Andreff << "\n";
//    //}
//    myfile_Andreff.close();

//    std::ofstream myfile_Dan;
//    myfile_Dan.open ("translation_Dan.csv",std::ios::app);
//    // for (int i = 0; i<perViewErrors.size(); i++)
//    //{
//    myfile_Dan << t_cam2gripper_DAN.t();
//    myfile_Dan << Rad2degV(Rodrigues_Dan).t();
//    myfile_Dan << "\n";
//    //}
//    myfile_Dan.close();



        // error estimation
    cout << endl << endl << "Error_TSAI: " << endl;
    errorEstimate(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,R_cam2gripper_TSAI,t_cam2gripper_TSAI);
    cout << endl << endl << "Error_HORAUD: " << endl;
    errorEstimate(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,R_cam2gripper_HORAUD,t_cam2gripper_HORAUD);
    cout << endl << endl << "Error_PARK: " << endl;
    errorEstimate(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,R_cam2gripper_PARK,t_cam2gripper_PARK);
    cout << endl << endl << "Error_ANDREFF: " << endl;
    errorEstimate(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,R_cam2gripper_ANDREFF,t_cam2gripper_ANDREFF);
    cout << endl << endl << "Error_DAN: " << endl;
    errorEstimate(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,R_cam2gripper_DAN,t_cam2gripper_DAN);

}

void print(std::vector<double> const &input)
{
    for (int i = 0; i < input.size(); i++) {
        std::cout << input.at(i) << ' ';
    }
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

Vec3f Rad2degV(Vec3f v)
{
    float x,y,z;
    x=v[0];
    y=v[1];
    z=v[2];
    return Vec3f(rad2deg(x), rad2deg(y), rad2deg(z));
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

Mat TransposeVector(Vec3d &v)
{

    Mat RV = Mat_<double>(3, 1, CV_64FC1);

    RV.at<double>(0,0)=v[0];
    RV.at<double>(0,1)=v[1];
    RV.at<double>(0,2)=v[2];

    return RV;
}

Vec3f Poly_Scale(Vec3f theta_rv)
{

float rv_len = sqrt(pow(theta_rv[0], 2) + pow(theta_rv[1], 2) + pow(theta_rv[2], 2));
float scale = 1 - 2 * CV_PI / rv_len;
theta_rv = theta_rv * scale;

return theta_rv;
}

std::vector<double> Pose_inv(std::vector<double> v)
{
    // Initialze
    cv::Mat t_Inverse(3, 1, CV_64F), R_Inverse(3, 1, CV_64F), R_Inverse_result(3, 1, CV_64F);
    cv::Mat Rodrigues_Inverse;
    cv::Mat t = (cv::Mat_<double>(3, 1)<< v[0],v[1],v[2]);
    cv::Mat R = (cv::Mat_<double>(3, 1)<< v[3],v[4],v[5]);

    //std::cout <<std::endl << "R " << std::endl << R << std::endl;
    //std::cout <<std::endl << "t " << std::endl << t << std::endl;

    // from 3x1 rotvec to 3x3 rotm
    cv::Rodrigues(R,Rodrigues_Inverse);

    //std::cout <<std::endl << "Rodrigues_Inverse " << std::endl << Rodrigues_Inverse << std::endl;

    // create inverse T
    R_Inverse=Rodrigues_Inverse.t();// Inverse Rotation
    t_Inverse= -R_Inverse * t; // Inverse translation

    //std::cout <<std::endl << "R_inverse " << std::endl << R_Inverse << std::endl;
    //std::cout <<std::endl << "t_Inverse " << std::endl << t_Inverse << std::endl;

    // from 3x3 rotvec to 3x1 rotm
    cv::Rodrigues(R_Inverse,R_Inverse_result);
    //std::cout <<std::endl << "R_Inverse_result " << std::endl << R_Inverse_result << std::endl;

    // cv::Mat --> std::vector
    std::vector<double>t_inv(t_Inverse.begin<double>(), t_Inverse.end<double>());
    std::vector<double>R_inv(R_Inverse_result.begin<double>(), R_Inverse_result.end<double>());

//    print(t_inv);
//    print(R_inv);

    std::vector<double> pose(t_inv);
        pose.insert(pose.end(), R_inv.begin(), R_inv.end());
        print(pose);
        std::cout << std::endl;
    return pose;
}

void errorEstimate(const std::vector<cv::Mat> &R_gripper2base, const std::vector<cv::Mat> &t_gripper2base, const std::vector<cv::Mat> &R_target2cam, const std::vector<cv::Mat> &t_target2cam, const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper)
{
    unsigned int nbPose = (unsigned int) R_target2cam.size();


    std::vector<cv::Mat> rMe(nbPose),cMo(nbPose);

    std::vector<vpHomogeneousMatrix> CmO(nbPose), RmE(nbPose);
    vpHomogeneousMatrix EmC;

    for (unsigned int i = 0; i < nbPose; i++) {

        // output the information of the matrices
        //      cout << "R_gripper2base = " << endl << " "  << R_gripper2base1[i] << endl << endl;
        //      cout << "R_target2cam = " << endl << " "  << R_target2cam1[i] << endl << endl;
        //      cout << "t_gripper2base = " << endl << " "  << t_gripper2base[i] << endl << endl;
        //      cout << "t_target2cam = " << endl << " "  << t_target2cam[i] << endl << endl;

        // concatenate the matrices
        cv::hconcat(R_gripper2base[i],t_gripper2base[i],rMe[i]);
        cv::hconcat(R_target2cam[i],t_target2cam[i],cMo[i]);
        //    cout << "rMe = " << endl << " "  << rMe[i] << endl << endl;
        //    cout << "cMo = " << endl << " "  << cMo[i] << endl << endl;

        // OpenCV conversion to VISP
        Mat2ViSP(cMo[i],CmO[i]);
        Mat2ViSP(rMe[i],RmE[i]);
//        cout << "RmE = " << endl << i << ": "   << RmE[i] << endl << endl;
//        cout << "CmO = " << endl << i << ": "   << CmO[i] << endl << endl;


    }


    cv::Mat eMc;
    cv::hconcat(R_cam2gripper,t_cam2gripper,eMc);
    //    cout << "eMc = " << endl << " "  << eMc << endl << endl;
    Mat2ViSP(eMc,EmC);
    //cout << "EmC = " << endl << " "  << EmC << endl << endl;

    // using the VISP libary on gathered data with OpenCV
    calibrationVerifrMo(CmO,RmE,EmC);
//    vpHomogeneousMatrix E;
//    vpHandEyeCalibration::calibrate(CmO,RmE,E);
//    std::cout << std::endl << "** Hand-eye calibration succeed" << std::endl;
//    std::cout << std::endl << "** Hand-eye (eMc) transformation estimated:" << std::endl;
//    std::cout << E << std::endl;
//    std::cout << "** Corresponding pose vector: " << vpPoseVector(E).t() << std::endl;

//    vpThetaUVector erc(E.getRotationMatrix());
//    std::cout << std::endl << "** Translation [m]: " << E[0][3] << " " << E[1][3] << " " << E[2][3] << std::endl;
//    std::cout << "** Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
//    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;

}

int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou)
{
  int ret = 0;

  //if (mat_in.type() != CV_32FC1 || mat_in.type() != CV_64FC1)
  if (mat_in.type() != CV_64FC1)
  {
    //std::cout << "[HandEyeCalib] Mat input is not floating-point number!" << std::endl;
    std::cout << "[HandEyeCalib] Mat input is not double floating-point number!" << std::endl;
    ret = 1;
    return ret;
  }

  for (int i=0; i<mat_in.rows; i++)
  {
    for (int j=0; j<mat_in.cols; j++)
    {
      visp_ou[i][j] = mat_in.ptr<double>(i)[j];  // new memory is created and data is copied in this line
//      cout << visp_ou[i][j] << endl;
    }
  }

  return ret;
}

void calibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe, const vpHomogeneousMatrix &eMc)
{
  unsigned int nbPose = (unsigned int) cMo.size();
  std::vector<vpTranslationVector> rTo(nbPose);
  std::vector<vpRotationMatrix> rRo(nbPose);

  for (unsigned int i = 0; i < nbPose; i++) {
    vpHomogeneousMatrix rMo = rMe[i] * eMc * cMo[i];
    rRo[i] = rMo.getRotationMatrix();
    rTo[i] = rMo.getTranslationVector();
  }
  vpRotationMatrix meanRot = vpRotationMatrix::mean(rRo);
  vpTranslationVector meanTrans = vpTranslationVector::mean(rTo);

#if DEBUG_LEVEL2
  {
    std::cout << "Mean  " << std::endl;
    std::cout << "Translation: " << meanTrans.t() << std::endl;
    vpThetaUVector P(meanRot);
    std::cout << "Rotation : theta (deg) = " << vpMath::deg(sqrt(P.sumSquare())) << " Matrice : " << std::endl << meanRot  << std::endl;
    std::cout << "theta U (deg): " << vpMath::deg(P[0]) << " " << vpMath::deg(P[1]) << " " << vpMath::deg(P[2]) << std::endl;
  }
#endif

  // standard deviation, rotational part
  double resRot = 0.0;
  for (unsigned int i = 0; i < nbPose; i++) {
    vpRotationMatrix R = meanRot.t() * rRo[i]; // Rm^T  Ri
    vpThetaUVector P(R);
    // theta = Riemannian distance d(Rm,Ri)
    double theta = sqrt(P.sumSquare());
    std::cout << "Distance theta between rMo(" << i << ") and mean (deg) = " << vpMath::deg(theta) << std::endl;
    // Euclidean distance d(Rm,Ri) not used
    // theta = 2.0*sqrt(2.0)*sin(theta/2.0);
    resRot += theta*theta;
  }
  resRot = sqrt(resRot/nbPose);
  std::cout << "Mean residual rMo(" << nbPose << ") - rotation (deg) = " << vpMath::deg(resRot) << std::endl;
  // standard deviation, translational part
  double resTrans = 0.0;
  for (unsigned int i = 0; i < nbPose; i++) {
    vpColVector errTrans = ((vpColVector) rTo[i]) - meanTrans;
    resTrans += errTrans.sumSquare();
    std::cout << "Distance d between rMo(" << i << ") and mean (m) = " << sqrt(errTrans.sumSquare()) << std::endl;
  }
  resTrans = sqrt(resTrans/nbPose);
  std::cout << "Mean residual rMo(" << nbPose << ") - translation (m) = " << resTrans << std::endl;
  double resPos = (resRot*resRot + resTrans*resTrans)*nbPose;
  resPos = sqrt(resPos/(2*nbPose));
  std::cout << "Mean residual rMo(" << nbPose << ") - global = " << resPos << std::endl;
}
