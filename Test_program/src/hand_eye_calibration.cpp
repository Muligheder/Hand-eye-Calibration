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
void print(std::vector<double> const &input);
int CHECKERBOARD[2]{6,9};

int main()
{
//****************************************************** CAMERA SETUP ************************************************************************************'//

    // Camera calibration information from API
    std::vector<double> distortionCoefficients(5);  // camera distortion
    distortionCoefficients[0] = 1.83375015854836e-01;
    distortionCoefficients[1] = -5.65327823162079e-01;
    distortionCoefficients[2] = -1.45305093610659e-04;
    distortionCoefficients[3] = 5.0213176291436e-04;
    distortionCoefficients[4] = 5.08288383483887e-01;

    float f_x = 6.8233544921875e+02; // Focal length in x axis
    float f_y = 6.82471923828125e+02; // Focal length in y axis
    float c_x = 483.771453857422; // Camera primary point x
    float c_y = 270.217803955078; // Camera primary point y

    // From calibration
//    std::vector<double> distortionCoefficients(5);  // camera distortion
//    distortionCoefficients[0] = 0.1536567074607056;
//    distortionCoefficients[1] = -0.4101400897139497;
//    distortionCoefficients[2] = 0.004554932098107907;
//    distortionCoefficients[3] = -0.0008190350641916984;
//    distortionCoefficients[4] = 0.2539752272062045;

//    float f_x = 689.4503417890605; // Focal length in x axis
//    float f_y = 688.716689747388; // Focal length in y axis
//    float c_x = 480.2259626431274; // Camera primary point x
//    float c_y = 280.8987271720027; // Camera primary point y

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

    cv::Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);

    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    cv::Mat R_cam2gripper = (cv::Mat_<float>(3, 3));
    cv::Mat t_cam2gripper_TSAI = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_HORAUD = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_PARK = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_ANDREFF = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_DAN = (cv::Mat_<float>(3, 1));


      // Creating vector to store vectors of 3D points for each checkerboard image
      std::vector<std::vector<cv::Point3f> > objpoints;

      // Creating vector to store vectors of 2D points for each checkerboard image
      std::vector<std::vector<cv::Point2f> > imgpoints;

      // Defining the world coordinates for 3D points
      std::vector<cv::Point3f> objp;

      // To calibrate reprojection error
      std::vector<double> stdDeviationsIntrinsics,stdDeviationExtrinsics,perViewErrors;
      cv::Mat rvecs,tvecs;

      for(int i{0}; i<CHECKERBOARD[1]; i++)
      {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
          objp.push_back(cv::Point3f(float(j),float(i),0.f));
       // cout << objp << endl;

      }

      // Extracting path of individual image stored in a given directory
      std::vector<cv::String> images;
      // Path of the folder containing checkerboard images
      std::string path = "/home/anders/Hand-eye-Calibration/Robot_control/workcell/Images_20/*.jpg";

      cv::glob(path, images);
      std::size_t num_images = images.size();
      cv::Mat frame, gray;

      R_gripper2base.reserve(num_images);
      t_gripper2base.reserve(num_images);
      R_target2cam.reserve(num_images);
      t_target2cam.reserve(num_images);

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
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
        */
        if(success)
        {
          cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.0001);

          // refining pixel coordinates for given 2d points.
          cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

          // Displaying the detected corner points on the checker board
          cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

          objpoints.push_back(objp);
          imgpoints.push_back(corner_pts);


          // Finding transformation matrix
          cv::solvePnP(cv::Mat(objp), cv::Mat(corner_pts), cameraMatrix, distortionCoefficients, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);

          cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distortionCoefficients, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationExtrinsics, perViewErrors,0);





          cv::Mat R;
                     cv::Rodrigues(rvec, R); // R is 3x3
                     R_target2cam.push_back(R);
                     t_target2cam.push_back(tvec*1);
                     cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
                     T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
                     T(cv::Range(0, 3), cv::Range(3, 4)) = tvec *1; // copies tvec into T

                     std::cout << "T = " << std::endl << " " << T << std::endl << std::endl;

        }

//        cv::imshow("Image",frame);
//        cv::waitKey(0);


      }

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
      std::cout << std::endl << "Size = "<< perViewErrors.size() << " " << "RMS re-projection error estimated for each pattern view = " << std::endl << std::endl;
      print(perViewErrors);
      std::cout<< std::endl << std::endl;
//*************************** OLD APPROACH *****************************'//

//    cv::Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);

//    std::vector<cv::Mat> R_gripper2base;
//    std::vector<cv::Mat> t_gripper2base;
//    std::vector<cv::Mat> R_target2cam;
//    std::vector<cv::Mat> t_target2cam;
//    cv::Mat R_cam2gripper = (cv::Mat_<float>(3, 3));
//    cv::Mat t_cam2gripper = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_TSAI = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_HORAUD = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_PARK = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_ANDREFF = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_DAN = (cv::Mat_<float>(3, 1));


//    std::vector<std::string> fn;
//    cv::glob("/home/anders/Hand-eye-Calibration/Robot_control/workcell/Images_20/*.jpg", fn, false);

//    std::vector<cv::Mat> images;
//    size_t num_images = fn.size(); //number of bmp files in images folder
//    std::cout << "number of images "<< num_images<<std::endl;
//    cv::Size patternsize(6, 9); //number of centers
//    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
//    double cell_size = 1;
//    std::vector<cv::Point3f> obj_points;

//    R_gripper2base.reserve(num_images);
//    t_gripper2base.reserve(num_images);
//    R_target2cam.reserve(num_images);
//    t_target2cam.reserve(num_images);

//    for (int i = 0; i < patternsize.height; ++i)
//        for (int j = 0; j < patternsize.width; ++j)
//            obj_points.push_back(cv::Point3f(float(j*cell_size),
//                                         float(i*cell_size), 0.f));

//cout <<"Objectpoints: " <<endl<<obj_points<<endl;
//cout <<"Objectpoints: " <<endl<<obj_points.size()<<endl;

//    for (size_t i = 0; i < num_images; i++)
//        images.push_back(cv::imread(fn[i]));


//    cv::Mat frame;

//    for (size_t i = 0; i < num_images; i++)
//    {
//        frame = cv::imread(fn[i]); //source image
//        cv::Mat gray;
//        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

////          cv::imshow("window",gray);
////         cv::waitKey(0);

//        bool patternfound = cv::findChessboardCorners(frame, patternsize, centers);
//        if (patternfound)
//        {
//            cornerSubPix(gray, centers, Size(1, 1), Size(-1, -1),
//                              TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

//            cv::drawChessboardCorners(frame, patternsize, cv::Mat(centers), patternfound);
//            // cv::imshow("window", frame);
//          // int key = cv::waitKey(0) & 0xff;
//            cv::solvePnP(cv::Mat(obj_points), cv::Mat(centers), cameraMatrix, distortionCoefficients, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);
//          //  cout <<"Rotation vetor = "<<endl<<" " << rvec<<endl<<endl;
//           // cout <<"Tranlation vetor = "<<endl<<" " << tvec<<endl<<endl;
//           // cout <<"Centers = "<<endl<<" " << centers<<endl<<endl;

//            cv::Mat R;
//            cv::Rodrigues(rvec, R); // R is 3x3
//            R_target2cam.push_back(R);
//            t_target2cam.push_back(tvec*1);
//            cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
//            T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
//            T(cv::Range(0, 3), cv::Range(3, 4)) = tvec *1; // copies tvec into T

//            std::cout << "T = " << std::endl << " " << T << std::endl << std::endl;

//        }
//        std::cout << patternfound << std::endl;
//    }


//****************************************************** CAMERA DONE ************************************************************************************//


    // read data from file and seperate to vector of vector.
           std::vector<std::vector<float>> v;
           std::ifstream in( "/home/anders/Hand-eye-Calibration/Robot_control/workcell/test_TCP.txt" );
           std::string record;

           while ( std::getline( in, record ) )
           {
               std::istringstream is( record );
               std::vector<float> row( ( std::istream_iterator<float>( is ) ),
                                        std::istream_iterator<float>() );
               v.push_back( row );
           }

//           for ( const auto &row : v )
//           {
//               for ( double x : row ) std::cout << x << ' ';
//               std::cout << std::endl;
//           }




    //Part2
    Vec3f theta_01{v[0][3],v[0][4],v[0][5]};
    Vec3f theta_02{v[1][3],v[1][4],v[1][5]};
    Vec3f theta_03{v[2][3],v[2][4],v[2][5]};
    Vec3f theta_04{v[3][3],v[3][4],v[3][5]};
    Vec3f theta_05{v[4][3],v[4][4],v[4][5]};
    Vec3f theta_06{v[5][3],v[5][4],v[5][5]};
    Vec3f theta_07{v[6][3],v[6][4],v[6][5]};
    Vec3f theta_08{v[7][3],v[7][4],v[7][5]};
    Vec3f theta_09{v[8][3],v[8][4],v[8][5]};
    Vec3f theta_10{v[9][3],v[9][4],v[9][5]};
    Vec3f theta_11{v[10][3],v[10][4],v[10][5]};
    Vec3f theta_12{v[11][3],v[11][4],v[11][5]};
    Vec3f theta_13{v[12][3],v[12][4],v[12][5]};
    Vec3f theta_14{v[13][3],v[13][4],v[13][5]};
    Vec3f theta_15{v[14][3],v[14][4],v[14][5]};
    Vec3f theta_16{v[15][3],v[15][4],v[15][5]};
    Vec3f theta_17{v[16][3],v[16][4],v[16][5]};
    Vec3f theta_18{v[17][3],v[17][4],v[17][5]};
    Vec3f theta_19{v[18][3],v[18][4],v[18][5]};
    Vec3f theta_20{v[19][3],v[19][4],v[19][5]};

    cout<< "theta 1" <<endl << theta_01<< endl << endl;
    cout<< "theta 2" <<endl << theta_02<< endl << endl;
    cout<< "theta 3" <<endl << theta_03<< endl << endl;
    cout<< "theta 4" <<endl << theta_04<< endl << endl;
    cout<< "theta 5" <<endl << theta_05<< endl << endl;
    cout<< "theta 6" <<endl << theta_06<< endl << endl;
    cout<< "theta 7" <<endl << theta_07<< endl << endl;
    cout<< "theta 8" <<endl << theta_08<< endl << endl;
    cout<< "theta 9" <<endl << theta_09<< endl << endl;
    cout<< "theta 10" <<endl << theta_10<< endl << endl;
    cout<< "theta 11" <<endl << theta_11<< endl << endl;
    cout<< "theta 12" <<endl << theta_12<< endl << endl;
    cout<< "theta 13" <<endl << theta_13<< endl << endl;
    cout<< "theta 14" <<endl << theta_14<< endl << endl;
    cout<< "theta 15" <<endl << theta_15<< endl << endl;
    cout<< "theta 16" <<endl << theta_16<< endl << endl;
    cout<< "theta 17" <<endl << theta_17<< endl << endl;
    cout<< "theta 18" <<endl << theta_18<< endl << endl;
    cout<< "theta 19" <<endl << theta_19<< endl << endl;
    cout<< "theta 20" <<endl << theta_20<< endl << endl;


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
    Mat robot_rot_11 = eulerAnglesToRotationMatrix(theta_11);
    Mat robot_rot_12 = eulerAnglesToRotationMatrix(theta_12);
    Mat robot_rot_13 = eulerAnglesToRotationMatrix(theta_13);
    Mat robot_rot_14 = eulerAnglesToRotationMatrix(theta_14);
    Mat robot_rot_15 = eulerAnglesToRotationMatrix(theta_15);
    Mat robot_rot_16 = eulerAnglesToRotationMatrix(theta_16);
    Mat robot_rot_17 = eulerAnglesToRotationMatrix(theta_17);
    Mat robot_rot_18 = eulerAnglesToRotationMatrix(theta_18);
    Mat robot_rot_19 = eulerAnglesToRotationMatrix(theta_19);
    Mat robot_rot_20 = eulerAnglesToRotationMatrix(theta_20);

    cout<< "robot_rot_01" <<endl << robot_rot_01<< endl << endl;
    cout<< "robot_rot_02" <<endl << robot_rot_02<< endl << endl;
    cout<< "robot_rot_03" <<endl << robot_rot_03<< endl << endl;
    cout<< "robot_rot_04" <<endl << robot_rot_04<< endl << endl;
    cout<< "robot_rot_05" <<endl << robot_rot_05<< endl << endl;
    cout<< "robot_rot_06" <<endl << robot_rot_06<< endl << endl;
    cout<< "robot_rot_07" <<endl << robot_rot_07<< endl << endl;
    cout<< "robot_rot_08" <<endl << robot_rot_08<< endl << endl;
    cout<< "robot_rot_09" <<endl << robot_rot_09<< endl << endl;
    cout<< "robot_rot_10" <<endl << robot_rot_10<< endl << endl;
    cout<< "robot_rot_11" <<endl << robot_rot_11<< endl << endl;
    cout<< "robot_rot_12" <<endl << robot_rot_12<< endl << endl;
    cout<< "robot_rot_13" <<endl << robot_rot_13<< endl << endl;
    cout<< "robot_rot_14" <<endl << robot_rot_14<< endl << endl;
    cout<< "robot_rot_15" <<endl << robot_rot_15<< endl << endl;
    cout<< "robot_rot_16" <<endl << robot_rot_16<< endl << endl;
    cout<< "robot_rot_17" <<endl << robot_rot_17<< endl << endl;
    cout<< "robot_rot_18" <<endl << robot_rot_18<< endl << endl;
    cout<< "robot_rot_19" <<endl << robot_rot_19<< endl << endl;
    cout<< "robot_rot_20" <<endl << robot_rot_20<< endl << endl;


    //Part 2
     Mat robot_tr_01 = (Mat_<float>(3, 1) << v[0][0]*1000,v[0][1]*1000,v[0][2]*1000);
     Mat robot_tr_02 = (Mat_<float>(3, 1) << v[1][0]*1000,v[1][1]*1000,v[1][2]*1000);
     Mat robot_tr_03 = (Mat_<float>(3, 1) << v[2][0]*1000,v[2][1]*1000,v[2][2]*1000);
     Mat robot_tr_04 = (Mat_<float>(3, 1) << v[3][0]*1000,v[3][1]*1000,v[3][2]*1000);
     Mat robot_tr_05 = (Mat_<float>(3, 1) << v[4][0]*1000,v[4][1]*1000,v[4][2]*1000);
     Mat robot_tr_06 = (Mat_<float>(3, 1) << v[5][0]*1000,v[5][1]*1000,v[5][2]*1000);
     Mat robot_tr_07 = (Mat_<float>(3, 1) << v[6][0]*1000,v[6][1]*1000,v[6][2]*1000);
     Mat robot_tr_08 = (Mat_<float>(3, 1) << v[7][0]*1000,v[7][1]*1000,v[7][2]*1000);
     Mat robot_tr_09 = (Mat_<float>(3, 1) << v[8][0]*1000,v[8][1]*1000,v[8][2]*1000);
     Mat robot_tr_10 = (Mat_<float>(3, 1) << v[9][0]*1000,v[9][1]*1000,v[9][2]*1000);
     Mat robot_tr_11 = (Mat_<float>(3, 1) << v[10][0]*1000,v[10][1]*1000,v[10][2]*1000);
     Mat robot_tr_12 = (Mat_<float>(3, 1) << v[11][0]*1000,v[11][1]*1000,v[11][2]*1000);
     Mat robot_tr_13 = (Mat_<float>(3, 1) << v[12][0]*1000,v[12][1]*1000,v[12][2]*1000);
     Mat robot_tr_14 = (Mat_<float>(3, 1) << v[13][0]*1000,v[13][1]*1000,v[13][2]*1000);
     Mat robot_tr_15 = (Mat_<float>(3, 1) << v[14][0]*1000,v[14][1]*1000,v[14][2]*1000);
     Mat robot_tr_16 = (Mat_<float>(3, 1) << v[15][0]*1000,v[15][1]*1000,v[15][2]*1000);
     Mat robot_tr_17 = (Mat_<float>(3, 1) << v[16][0]*1000,v[16][1]*1000,v[16][2]*1000);
     Mat robot_tr_18 = (Mat_<float>(3, 1) << v[17][0]*1000,v[17][1]*1000,v[17][2]*1000);
     Mat robot_tr_19 = (Mat_<float>(3, 1) << v[18][0]*1000,v[18][1]*1000,v[18][2]*1000);
     Mat robot_tr_20 = (Mat_<float>(3, 1) << v[19][0]*1000,v[19][1]*1000,v[19][2]*1000);

     cout<< "Translation 1" <<endl << robot_tr_01<< endl << endl;
     cout<< "Translation 2" <<endl << robot_tr_02<< endl << endl;
     cout<< "Translation 3" <<endl << robot_tr_03<< endl << endl;
     cout<< "Translation 4" <<endl << robot_tr_04<< endl << endl;
     cout<< "Translation 5" <<endl << robot_tr_05<< endl << endl;
     cout<< "Translation 6" <<endl << robot_tr_06<< endl << endl;
     cout<< "Translation 7" <<endl << robot_tr_07<< endl << endl;
     cout<< "Translation 8" <<endl << robot_tr_08<< endl << endl;
     cout<< "Translation 9" <<endl << robot_tr_09<< endl << endl;
     cout<< "Translation 10" <<endl << robot_tr_10<< endl << endl;
     cout<< "Translation 11" <<endl << robot_tr_11<< endl << endl;
     cout<< "Translation 12" <<endl << robot_tr_12<< endl << endl;
     cout<< "Translation 13" <<endl << robot_tr_13<< endl << endl;
     cout<< "Translation 14" <<endl << robot_tr_14<< endl << endl;
     cout<< "Translation 15" <<endl << robot_tr_15<< endl << endl;
     cout<< "Translation 16" <<endl << robot_tr_16<< endl << endl;
     cout<< "Translation 17" <<endl << robot_tr_17<< endl << endl;
     cout<< "Translation 18" <<endl << robot_tr_18<< endl << endl;
     cout<< "Translation 19" <<endl << robot_tr_19<< endl << endl;
     cout<< "Translation 20" <<endl << robot_tr_20<< endl << endl;


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
    R_gripper2base.push_back(robot_rot_11);
    R_gripper2base.push_back(robot_rot_12);
    R_gripper2base.push_back(robot_rot_13);
    R_gripper2base.push_back(robot_rot_14);
    R_gripper2base.push_back(robot_rot_15);
    R_gripper2base.push_back(robot_rot_16);
    R_gripper2base.push_back(robot_rot_17);
    R_gripper2base.push_back(robot_rot_18);
    R_gripper2base.push_back(robot_rot_19);
    R_gripper2base.push_back(robot_rot_20);



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
    t_gripper2base.push_back(robot_tr_11);
    t_gripper2base.push_back(robot_tr_12);
    t_gripper2base.push_back(robot_tr_13);
    t_gripper2base.push_back(robot_tr_14);
    t_gripper2base.push_back(robot_tr_15);
    t_gripper2base.push_back(robot_tr_16);
    t_gripper2base.push_back(robot_tr_17);
    t_gripper2base.push_back(robot_tr_18);
    t_gripper2base.push_back(robot_tr_19);
    t_gripper2base.push_back(robot_tr_20);

    //cout << "R_gripper2base = " << endl << " " << R_gripper2base[0] << endl << endl;
    //cout << "t_gripper2base = " << endl << " " << t_gripper2base[0] << endl << endl;



    cout << R_gripper2base.size() << " R_gripper2base"<<endl;
    cout << t_gripper2base.size() << " t_gripper2base"<<endl;
    cout << R_target2cam.size() << " R_target2cam"<<endl;
    cout << R_target2cam.size() << " t_target2cam"<<endl;
    cout << R_cam2gripper.size() << " R_cam2gripper"<< endl << endl;
    //cout << t_cam2gripper.size() << " t_cam2gripper"<<endl<<endl;

//    cout << "Input to calibrateHandEyed: "<<endl;
//    cout << "R_gripper2base = "<< endl <<" " << R_gripper2base[4] << endl<< endl;
//    cout << "t_gripper2base = "<< endl <<" " << t_gripper2base[4] << endl<< endl;
//    cout << "R_target2cam = "<< endl <<" " << R_target2cam[4] << endl<< endl;
//    cout << "t_target2cam = "<< endl <<" " << t_target2cam[4] << endl<< endl;





    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper_TSAI, CALIB_HAND_EYE_TSAI);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper_HORAUD, CALIB_HAND_EYE_HORAUD);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper_ANDREFF, CALIB_HAND_EYE_ANDREFF);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper_PARK, CALIB_HAND_EYE_PARK);
    calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper_DAN, CALIB_HAND_EYE_DANIILIDIS);
    Vec3f R_cam2gripper_r = rotationMatrixToEulerAngles(R_cam2gripper);

    cout << "R_cam2gripper = " << endl << " " << R_cam2gripper << endl << endl;
    cout << "R_cam2gripper_r = " << endl << " " << R_cam2gripper_r << endl << endl;
    cout << "t_cam2gripper_TSAI = " << endl << " " << t_cam2gripper_TSAI << endl << endl;
    cout << "t_cam2gripper_HORAUD = " << endl << " " << t_cam2gripper_HORAUD << endl << endl;
    cout << "t_cam2gripper_PARK = " << endl << " " << t_cam2gripper_PARK << endl << endl;
    cout << "t_cam2gripper_ANDREFF = " << endl << " " << t_cam2gripper_ANDREFF << endl << endl;
    cout << "t_cam2gripper_DAN = " << endl << " " << t_cam2gripper_DAN << endl << endl;
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
