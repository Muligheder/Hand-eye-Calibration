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

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <numeric>

using namespace cv;
using namespace std;

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
Vec3f rotationMatrixToEulerAngles(Mat &R);
float rad2deg(float radian);
float deg2rad(float degree);
Mat ReverseVector(Mat &v);
void print(std::vector<double> const &input);
Vec3f Rad2degV(Vec3f v);
Vec3f Poly_Scale(Vec3f theta_rv);
Mat TransposeVector(Vec3f &v);
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

//    // Camera calibration information from calibration 1920x1080
//    std::vector<double> distortionCoefficients(5);  // camera distortion
//    distortionCoefficients[0] = 0.158507;
//    distortionCoefficients[1] = -0.488571;
//    distortionCoefficients[2] = 0.00529788 ;
//    distortionCoefficients[3] = 0.000712291;
//    distortionCoefficients[4] = 0.426095;

//    float f_x = 1369.41272730028; // Focal length in x axis
//    float f_y = 1368.515018183664; // Focal length in y axis
//    float c_x = 965.5782226153243; // Camera primary point x
//    float c_y = 560.7514324578995; // Camera primary point y


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

    cv::Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);

    std::vector<cv::Mat> R_gripper2base;
    //std::vector<cv::Mat> R_gripper2base_Euler;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    cv::Mat R_cam2gripper_TSAI = (cv::Mat_<float>(3, 3));
    cv::Mat R_cam2gripper_HORAUD = (cv::Mat_<float>(3, 3));
    cv::Mat R_cam2gripper_PARK = (cv::Mat_<float>(3, 3));
    cv::Mat R_cam2gripper_ANDREFF = (cv::Mat_<float>(3, 3));
    cv::Mat R_cam2gripper_DAN = (cv::Mat_<float>(3, 3));
    cv::Mat t_cam2gripper_TSAI = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_HORAUD = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_PARK = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_ANDREFF = (cv::Mat_<float>(3, 1));
    cv::Mat t_cam2gripper_DAN = (cv::Mat_<float>(3, 1));

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
      float Cell_size = 30;

      for(int i{0}; i<CHECKERBOARD[1]; i++)
      {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
          objp.push_back(cv::Point3f(float(j*Cell_size),float(i*Cell_size),0.f));
        std::cout << objp << std::endl;

      }

      // Extracting path of individual image stored in a given directory
      std::vector<cv::String> images;
      // Path of the folder containing checkerboard images
      std::string path = "/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/Images/*.bmp";

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
          cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);

          // refining pixel coordinates for given 2d points.
          cv::cornerSubPix(gray,corner_pts,cv::Size(5,5), cv::Size(-1,-1),criteria);

          // Displaying the detected corner points on the checker board
          cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

          objpoints.push_back(objp);
          imgpoints.push_back(corner_pts);


          // Finding transformation matrix
          cv::solvePnP(cv::Mat(objp), cv::Mat(corner_pts), cameraMatrix, distortionCoefficients, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);

          cv::Mat R;
                     cv::Rodrigues(rvec, R); // R is 3x3
                     std::cout <<std::endl << "Rvec " << std::endl << rvec << std::endl;
                     R_target2cam.push_back(R);
                     t_target2cam.push_back(tvec);
                     cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
                     T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
                     T(cv::Range(0, 3), cv::Range(3, 4)) = tvec *1; // copies tvec into T

                     std::cout << "T = " << std::endl << " " << T << std::endl << std::endl;

        }

//        cv::imshow("Image",frame);
//        cv::waitKey(0);



      }
      float Avg_RMS;
      cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distortionCoefficients, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationExtrinsics, perViewErrors,0);
      //float Avg_RMS = std::accumulate(perViewErrors.begin(), perViewErrors.end(), 0);
      for (auto& n : perViewErrors)
          Avg_RMS += n;

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
//*************************** OLD APPROACH *****************************//

//    cv::Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);

//    std::vector<cv::Mat> R_gripper2base;
//    std::vector<cv::Mat> t_gripper2base;
//    std::vector<cv::Mat> R_target2cam;
//    std::vector<cv::Mat> t_target2cam;
//    cv::Mat R_cam2gripper_TSAI = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_HORAUD = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_PARK = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_ANDREFF = (cv::Mat_<float>(3, 3));
//    cv::Mat R_cam2gripper_DAN = (cv::Mat_<float>(3, 3));
//    cv::Mat t_cam2gripper_TSAI = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_HORAUD = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_PARK = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_ANDREFF = (cv::Mat_<float>(3, 1));
//    cv::Mat t_cam2gripper_DAN = (cv::Mat_<float>(3, 1));


//    std::vector<std::string> fn;
//    cv::glob("/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/Images/*.bmp", fn, false);

//    std::vector<cv::Mat> images;
//    size_t num_images = fn.size(); //number of bmp files in images folder
//    std::cout << "number of images "<< num_images<<std::endl;
//    cv::Size patternsize(5, 8); //number of centers
//    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
//    double cell_size = 30;
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
           std::ifstream in( "/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/test.txt" );
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




//    //Part2        x       y       z
//    Vec3f theta_01{v[0][3],v[0][4],v[0][5]};
//    Vec3f theta_02{v[1][3],v[1][4],v[1][5]};
//    Vec3f theta_03{v[2][3],v[2][4],v[2][5]};
//    Vec3f theta_04{v[3][3],v[3][4],v[3][5]};
//    Vec3f theta_05{v[4][3],v[4][4],v[4][5]};
//    Vec3f theta_06{v[5][3],v[5][4],v[5][5]};
//    Vec3f theta_07{v[6][3],v[6][4],v[6][5]};
//    Vec3f theta_08{v[7][3],v[7][4],v[7][5]};
//    Vec3f theta_09{v[8][3],v[8][4],v[8][5]};
//    Vec3f theta_10{v[9][3],v[9][4],v[9][5]};




// With Rodriguez method  x       y       z
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
//    cv::Mat theta_11{v[10][3],v[10][4],v[10][5]};
//    cv::Mat theta_12{v[11][3],v[11][4],v[11][5]};
//    cv::Mat theta_13{v[12][3],v[12][4],v[12][5]};
//    cv::Mat theta_14{v[13][3],v[13][4],v[13][5]};
//    cv::Mat theta_15{v[14][3],v[14][4],v[14][5]};
//    cv::Mat theta_16{v[15][3],v[15][4],v[15][5]};
//    cv::Mat theta_17{v[16][3],v[16][4],v[16][5]};
//    cv::Mat theta_18{v[17][3],v[17][4],v[17][5]};
//    cv::Mat theta_19{v[18][3],v[18][4],v[18][5]};
//    cv::Mat theta_20{v[19][3],v[19][4],v[19][5]};
//    cv::Mat theta_21{v[20][3],v[20][4],v[20][5]};
//    cv::Mat theta_22{v[21][3],v[21][4],v[21][5]};
//    cv::Mat theta_23{v[22][3],v[22][4],v[22][5]};

    cout<< "Before scaling: " <<endl << endl;
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

        theta_01=Poly_Scale(theta_01);
        theta_02=Poly_Scale(theta_02);
        theta_03=Poly_Scale(theta_03);
        theta_04=Poly_Scale(theta_04);
        theta_05=Poly_Scale(theta_05);
        theta_06=Poly_Scale(theta_06);
        theta_07=Poly_Scale(theta_07);
        theta_08=Poly_Scale(theta_08);
        theta_09=Poly_Scale(theta_09);
        theta_10=Poly_Scale(theta_10);


    cv::Mat theta_scaled_01 = TransposeVector(theta_01);
    cv::Mat theta_scaled_02 = TransposeVector(theta_02);
    cv::Mat theta_scaled_03 = TransposeVector(theta_03);
    cv::Mat theta_scaled_04 = TransposeVector(theta_04);
    cv::Mat theta_scaled_05 = TransposeVector(theta_05);
    cv::Mat theta_scaled_06 = TransposeVector(theta_06);
    cv::Mat theta_scaled_07 = TransposeVector(theta_07);
    cv::Mat theta_scaled_08 = TransposeVector(theta_08);
    cv::Mat theta_scaled_09 = TransposeVector(theta_09);
    cv::Mat theta_scaled_10 = TransposeVector(theta_10);

//        cv::Mat theta_011=Poly_Scale(theta_01);
//       cv::Mat theta_021=Poly_Scale(theta_02);
//       cv::Mat theta_031=Poly_Scale(theta_03);
//       cv::Mat theta_041=Poly_Scale(theta_04);
//       cv::Mat theta_051=Poly_Scale(theta_05);
//       cv::Mat theta_061=Poly_Scale(theta_06);
//       cv::Mat theta_071=Poly_Scale(theta_07);
//       cv::Mat theta_081=Poly_Scale(theta_08);
//       cv::Mat theta_091=Poly_Scale(theta_09);
//       cv::Mat theta_101=Poly_Scale(theta_10);


    cv::Mat robot_rot_01,robot_rot_02,robot_rot_03,robot_rot_04,robot_rot_05,robot_rot_06,robot_rot_07,robot_rot_08,robot_rot_09,robot_rot_10,robot_rot_11,robot_rot_12,robot_rot_13,robot_rot_14,robot_rot_15,robot_rot_16,robot_rot_17,robot_rot_18,robot_rot_19,robot_rot_20,robot_rot_21,robot_rot_22,robot_rot_23 ;
    cv::Rodrigues(theta_scaled_01,robot_rot_01);
    cv::Rodrigues(theta_scaled_02,robot_rot_02);
    cv::Rodrigues(theta_scaled_03,robot_rot_03);
    cv::Rodrigues(theta_scaled_04,robot_rot_04);
    cv::Rodrigues(theta_scaled_05,robot_rot_05);
    cv::Rodrigues(theta_scaled_06,robot_rot_06);
    cv::Rodrigues(theta_scaled_07,robot_rot_07);
    cv::Rodrigues(theta_scaled_08,robot_rot_08);
    cv::Rodrigues(theta_scaled_09,robot_rot_09);
    cv::Rodrigues(theta_scaled_10,robot_rot_10);
//    cv::Rodrigues(theta_11,robot_rot_11);
//    cv::Rodrigues(theta_12,robot_rot_12);
//    cv::Rodrigues(theta_13,robot_rot_13);
//    cv::Rodrigues(theta_14,robot_rot_14);
//    cv::Rodrigues(theta_15,robot_rot_15);
//    cv::Rodrigues(theta_16,robot_rot_16);
//    cv::Rodrigues(theta_17,robot_rot_17);
//    cv::Rodrigues(theta_18,robot_rot_18);
//    cv::Rodrigues(theta_19,robot_rot_19);
//    cv::Rodrigues(theta_20,robot_rot_20);
//    cv::Rodrigues(theta_21,robot_rot_21);
//    cv::Rodrigues(theta_22,robot_rot_22);
//    cv::Rodrigues(theta_23,robot_rot_23);

    cout<< "After scaling: " <<endl << endl;
    cout<< "theta 1" <<endl << theta_scaled_01<< endl << endl;
    cout<< "theta 2" <<endl << theta_scaled_02<< endl << endl;
    cout<< "theta 3" <<endl << theta_scaled_03<< endl << endl;
    cout<< "theta 4" <<endl << theta_scaled_04<< endl << endl;
    cout<< "theta 5" <<endl << theta_scaled_05<< endl << endl;
    cout<< "theta 6" <<endl << theta_scaled_06<< endl << endl;
    cout<< "theta 7" <<endl << theta_scaled_07<< endl << endl;
    cout<< "theta 8" <<endl << theta_scaled_08<< endl << endl;
    cout<< "theta 9" <<endl << theta_scaled_09<< endl << endl;
    cout<< "theta 10" <<endl << theta_scaled_10<< endl << endl;
//    cout<< "theta 11" <<endl << theta_11<< endl << endl;
//    cout<< "theta 12" <<endl << theta_12<< endl << endl;
//    cout<< "theta 13" <<endl << theta_13<< endl << endl;
//    cout<< "theta 14" <<endl << theta_14<< endl << endl;
//    cout<< "theta 15" <<endl << theta_15<< endl << endl;
//    cout<< "theta 16" <<endl << theta_16<< endl << endl;
//    cout<< "theta 17" <<endl << theta_17<< endl << endl;
//    cout<< "theta 18" <<endl << theta_18<< endl << endl;
//    cout<< "theta 19" <<endl << theta_19<< endl << endl;
//    cout<< "theta 20" <<endl << theta_20<< endl << endl;
//    cout<< "theta 21" <<endl << theta_21<< endl << endl;
//    cout<< "theta 22" <<endl << theta_22<< endl << endl;
//    cout<< "theta 23" <<endl << theta_23<< endl << endl;
//    cout<< "theta 24" <<endl << theta_24<< endl << endl;
//    cout<< "theta 25" <<endl << theta_25<< endl << endl;
//    cout<< "theta 26" <<endl << theta_26<< endl << endl;
//    cout<< "theta 27" <<endl << theta_27<< endl << endl;
//    cout<< "theta 28" <<endl << theta_28<< endl << endl;
//    cout<< "theta 29" <<endl << theta_29<< endl << endl;
//    cout<< "theta 30" <<endl << theta_30<< endl << endl;
//    cout<< "theta 31" <<endl << theta_31<< endl << endl;
//    cout<< "theta 32" <<endl << theta_32<< endl << endl;
//    cout<< "theta 33" <<endl << theta_33<< endl << endl;
//    cout<< "theta 34" <<endl << theta_34<< endl << endl;
//    cout<< "theta 35" <<endl << theta_35<< endl << endl;
//    cout<< "theta 36" <<endl << theta_36<< endl << endl;
//    cout<< "theta 37" <<endl << theta_37<< endl << endl;
//    cout<< "theta 38" <<endl << theta_38<< endl << endl;
//    cout<< "theta 39" <<endl << theta_39<< endl << endl;
//    cout<< "theta 40" <<endl << theta_40<< endl << endl;


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
//    cout<< "robot_rot_11" <<endl << robot_rot_11<< endl << endl;
//    cout<< "robot_rot_12" <<endl << robot_rot_12<< endl << endl;
//    cout<< "robot_rot_13" <<endl << robot_rot_13<< endl << endl;
//    cout<< "robot_rot_14" <<endl << robot_rot_14<< endl << endl;
//    cout<< "robot_rot_15" <<endl << robot_rot_15<< endl << endl;
//    cout<< "robot_rot_16" <<endl << robot_rot_16<< endl << endl;
//    cout<< "robot_rot_17" <<endl << robot_rot_17<< endl << endl;
//    cout<< "robot_rot_18" <<endl << robot_rot_18<< endl << endl;
//    cout<< "robot_rot_19" <<endl << robot_rot_19<< endl << endl;
//    cout<< "robot_rot_20" <<endl << robot_rot_20<< endl << endl;
//    cout<< "robot_rot_21" <<endl << robot_rot_21<< endl << endl;
//    cout<< "robot_rot_22" <<endl << robot_rot_22<< endl << endl;
//    cout<< "robot_rot_23" <<endl << robot_rot_23<< endl << endl;
//    cout<< "robot_rot_24" <<endl << robot_rot_24<< endl << endl;
//    cout<< "robot_rot_25" <<endl << robot_rot_25<< endl << endl;
//    cout<< "robot_rot_26" <<endl << robot_rot_26<< endl << endl;
//    cout<< "robot_rot_27" <<endl << robot_rot_27<< endl << endl;
//    cout<< "robot_rot_28" <<endl << robot_rot_28<< endl << endl;
//    cout<< "robot_rot_29" <<endl << robot_rot_29<< endl << endl;
//    cout<< "robot_rot_30" <<endl << robot_rot_30<< endl << endl;
//    cout<< "robot_rot_31" <<endl << robot_rot_31<< endl << endl;
//    cout<< "robot_rot_32" <<endl << robot_rot_32<< endl << endl;
//    cout<< "robot_rot_33" <<endl << robot_rot_33<< endl << endl;
//    cout<< "robot_rot_34" <<endl << robot_rot_34<< endl << endl;
//    cout<< "robot_rot_35" <<endl << robot_rot_35<< endl << endl;
//    cout<< "robot_rot_36" <<endl << robot_rot_36<< endl << endl;
//    cout<< "robot_rot_37" <<endl << robot_rot_37<< endl << endl;
//    cout<< "robot_rot_38" <<endl << robot_rot_38<< endl << endl;
//    cout<< "robot_rot_39" <<endl << robot_rot_39<< endl << endl;
//    cout<< "robot_rot_40" <<endl << robot_rot_40<< endl << endl;

//    cv::Vec3f thetaa_01{v[0][3],v[0][4],v[0][5]};
//    cv::Vec3f thetaa_02{v[1][3],v[1][4],v[1][5]};
//    cv::Vec3f thetaa_03{v[2][3],v[2][4],v[2][5]};
//    cv::Vec3f thetaa_04{v[3][3],v[3][4],v[3][5]};
//    cv::Vec3f thetaa_05{v[4][3],v[4][4],v[4][5]};
//    cv::Vec3f thetaa_06{v[5][3],v[5][4],v[5][5]};
//    cv::Vec3f thetaa_07{v[6][3],v[6][4],v[6][5]};
//    cv::Vec3f thetaa_08{v[7][3],v[7][4],v[7][5]};
//    cv::Vec3f thetaa_09{v[8][3],v[8][4],v[8][5]};
//    cv::Vec3f thetaa_10{v[9][3],v[9][4],v[9][5]};
//    cv::Vec3f thetaa_11{v[10][5],v[10][4],v[10][3]};
//    cv::Vec3f thetaa_12{v[11][5],v[11][4],v[11][3]};
//    cv::Vec3f thetaa_13{v[12][5],v[12][4],v[12][3]};
//    cv::Vec3f thetaa_14{v[13][5],v[13][4],v[13][3]};
//    cv::Vec3f thetaa_15{v[14][5],v[14][4],v[14][3]};
//    cv::Vec3f thetaa_16{v[15][5],v[15][4],v[15][3]};
//    cv::Vec3f thetaa_17{v[16][5],v[16][4],v[16][3]};
//    cv::Vec3f thetaa_18{v[17][5],v[17][4],v[17][3]};


//    Mat robotE_rot_01 = eulerAnglesToRotationMatrix(thetaa_01);
//    Mat robotE_rot_02 = eulerAnglesToRotationMatrix(thetaa_02);
//    Mat robotE_rot_03 = eulerAnglesToRotationMatrix(thetaa_03);
//    Mat robotE_rot_04 = eulerAnglesToRotationMatrix(thetaa_04);
//    Mat robotE_rot_05 = eulerAnglesToRotationMatrix(thetaa_05);
//    Mat robotE_rot_06 = eulerAnglesToRotationMatrix(thetaa_06);
//    Mat robotE_rot_07 = eulerAnglesToRotationMatrix(thetaa_07);
//    Mat robotE_rot_08 = eulerAnglesToRotationMatrix(thetaa_08);
//    Mat robotE_rot_09 = eulerAnglesToRotationMatrix(thetaa_09);
//    Mat robotE_rot_10 = eulerAnglesToRotationMatrix(thetaa_10);
//    Mat robotE_rot_11 = eulerAnglesToRotationMatrix(thetaa_11);
//    Mat robotE_rot_12 = eulerAnglesToRotationMatrix(thetaa_12);
//    Mat robotE_rot_13 = eulerAnglesToRotationMatrix(thetaa_13);
//    Mat robotE_rot_14 = eulerAnglesToRotationMatrix(thetaa_14);
//    Mat robotE_rot_15 = eulerAnglesToRotationMatrix(thetaa_15);
//    Mat robotE_rot_16 = eulerAnglesToRotationMatrix(thetaa_16);
//    Mat robotE_rot_17 = eulerAnglesToRotationMatrix(thetaa_17);
//    Mat robotE_rot_18 = eulerAnglesToRotationMatrix(thetaa_18);

//    cout<< "thetaa 1" <<endl << thetaa_01<< endl << endl;
//    cout<< "thetaa 2" <<endl << thetaa_02<< endl << endl;
//    cout<< "thetaa 3" <<endl << thetaa_03<< endl << endl;
//    cout<< "thetaa 4" <<endl << thetaa_04<< endl << endl;
//    cout<< "thetaa 5" <<endl << thetaa_05<< endl << endl;
//    cout<< "thetaa 6" <<endl << thetaa_06<< endl << endl;
//    cout<< "thetaa 7" <<endl << thetaa_07<< endl << endl;
//    cout<< "thetaa 8" <<endl << thetaa_08<< endl << endl;
//    cout<< "thetaa 9" <<endl << thetaa_09<< endl << endl;
//    cout<< "thetaa 10" <<endl << thetaa_10<< endl << endl;
//    cout<< "thetaa 11" <<endl << thetaa_11<< endl << endl;
//    cout<< "thetaa 12" <<endl << thetaa_12<< endl << endl;
//    cout<< "thetaa 13" <<endl << thetaa_13<< endl << endl;
//    cout<< "thetaa 14" <<endl << thetaa_14<< endl << endl;
//    cout<< "thetaa 15" <<endl << thetaa_15<< endl << endl;
//    cout<< "thetaa 16" <<endl << thetaa_16<< endl << endl;
//    cout<< "thetaa 17" <<endl << thetaa_17<< endl << endl;
//    cout<< "thetaa 18" <<endl << thetaa_18<< endl << endl;


//    cout<< "robotE_rot_01" <<endl << robotE_rot_01<< endl << endl;
//    cout<< "robotE_rot_02" <<endl << robotE_rot_02<< endl << endl;
//    cout<< "robotE_rot_03" <<endl << robotE_rot_03<< endl << endl;
//    cout<< "robotE_rot_04" <<endl << robotE_rot_04<< endl << endl;
//    cout<< "robotE_rot_05" <<endl << robotE_rot_05<< endl << endl;
//    cout<< "robotE_rot_06" <<endl << robotE_rot_06<< endl << endl;
//    cout<< "robotE_rot_07" <<endl << robotE_rot_07<< endl << endl;
//    cout<< "robotE_rot_08" <<endl << robotE_rot_08<< endl << endl;
//    cout<< "robotE_rot_09" <<endl << robotE_rot_09<< endl << endl;
//    cout<< "robotE_rot_10" <<endl << robotE_rot_10<< endl << endl;
//    cout<< "robotE_rot_11" <<endl << robotE_rot_11<< endl << endl;
//    cout<< "robotE_rot_12" <<endl << robotE_rot_12<< endl << endl;
//    cout<< "robotE_rot_13" <<endl << robotE_rot_13<< endl << endl;
//    cout<< "robotE_rot_14" <<endl << robotE_rot_14<< endl << endl;
//    cout<< "robotE_rot_15" <<endl << robotE_rot_15<< endl << endl;
//    cout<< "robotE_rot_16" <<endl << robotE_rot_16<< endl << endl;
//    cout<< "robotE_rot_17" <<endl << robotE_rot_17<< endl << endl;
//    cout<< "robotE_rot_18" <<endl << robotE_rot_18<< endl << endl;
//    cout<< "robot_rot_19" <<endl << robot_rot_19<< endl << endl;
//    cout<< "robot_rot_20" <<endl << robot_rot_20<< endl << endl;
//    cout<< "robot_rot_21" <<endl << robot_rot_21<< endl << endl;
//    cout<< "robot_rot_22" <<endl << robot_rot_22<< endl << endl;
//    cout<< "robot_rot_23" <<endl << robot_rot_23<< endl << endl;
//    cout<< "robot_rot_24" <<endl << robot_rot_24<< endl << endl;
//    cout<< "robot_rot_25" <<endl << robot_rot_25<< endl << endl;
//    cout<< "robot_rot_26" <<endl << robot_rot_26<< endl << endl;
//    cout<< "robot_rot_27" <<endl << robot_rot_27<< endl << endl;
//    cout<< "robot_rot_28" <<endl << robot_rot_28<< endl << endl;
//    cout<< "robot_rot_29" <<endl << robot_rot_29<< endl << endl;
//    cout<< "robot_rot_30" <<endl << robot_rot_30<< endl << endl;
//    cout<< "robot_rot_31" <<endl << robot_rot_31<< endl << endl;
//    cout<< "robot_rot_32" <<endl << robot_rot_32<< endl << endl;
//    cout<< "robot_rot_33" <<endl << robot_rot_33<< endl << endl;
//    cout<< "robot_rot_34" <<endl << robot_rot_34<< endl << endl;
//    cout<< "robot_rot_35" <<endl << robot_rot_35<< endl << endl;
//    cout<< "robot_rot_36" <<endl << robot_rot_36<< endl << endl;
//    cout<< "robot_rot_37" <<endl << robot_rot_37<< endl << endl;
//    cout<< "robot_rot_38" <<endl << robot_rot_38<< endl << endl;
//    cout<< "robot_rot_39" <<endl << robot_rot_39<< endl << endl;
//    cout<< "robot_rot_40" <<endl << robot_rot_40<< endl << endl;
    //Part 2                                      X            Y            Z
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
//     Mat robot_tr_11 = (Mat_<float>(3, 1) << v[10][0]*1000,v[10][1]*1000,v[10][2]*1000);
//     Mat robot_tr_12 = (Mat_<float>(3, 1) << v[11][0]*1000,v[11][1]*1000,v[11][2]*1000);
//     Mat robot_tr_13 = (Mat_<float>(3, 1) << v[12][0]*1000,v[12][1]*1000,v[12][2]*1000);
//     Mat robot_tr_14 = (Mat_<float>(3, 1) << v[13][0]*1000,v[13][1]*1000,v[13][2]*1000);
//     Mat robot_tr_15 = (Mat_<float>(3, 1) << v[14][0]*1000,v[14][1]*1000,v[14][2]*1000);
//     Mat robot_tr_16 = (Mat_<float>(3, 1) << v[15][0]*1000,v[15][1]*1000,v[15][2]*1000);
//     Mat robot_tr_17 = (Mat_<float>(3, 1) << v[16][0]*1000,v[16][1]*1000,v[16][2]*1000);
//     Mat robot_tr_18 = (Mat_<float>(3, 1) << v[17][0]*1000,v[17][1]*1000,v[17][2]*1000);
//     Mat robot_tr_19 = (Mat_<float>(3, 1) << v[18][0]*1000,v[18][1]*1000,v[18][2]*1000);
//     Mat robot_tr_20 = (Mat_<float>(3, 1) << v[19][0]*1000,v[19][1]*1000,v[19][2]*1000);
//     Mat robot_tr_21 = (Mat_<float>(3, 1) << v[20][0]*1000,v[20][1]*1000,v[20][2]*1000);
//     Mat robot_tr_22 = (Mat_<float>(3, 1) << v[21][0]*1000,v[21][1]*1000,v[21][2]*1000);
//     Mat robot_tr_23 = (Mat_<float>(3, 1) << v[22][0]*1000,v[22][1]*1000,v[22][2]*1000);
//     Mat robot_tr_24 = (Mat_<float>(3, 1) << v[23][0]*1000,v[23][1]*1000,v[23][2]*1000);
//     Mat robot_tr_25 = (Mat_<float>(3, 1) << v[24][0]*1000,v[24][1]*1000,v[24][2]*1000);
//     Mat robot_tr_26 = (Mat_<float>(3, 1) << v[25][0]*1000,v[25][1]*1000,v[25][2]*1000);
//     Mat robot_tr_27 = (Mat_<float>(3, 1) << v[26][0]*1000,v[26][1]*1000,v[26][2]*1000);
//     Mat robot_tr_28 = (Mat_<float>(3, 1) << v[27][0]*1000,v[27][1]*1000,v[27][2]*1000);
//     Mat robot_tr_29 = (Mat_<float>(3, 1) << v[28][0]*1000,v[28][1]*1000,v[28][2]*1000);
//     Mat robot_tr_30 = (Mat_<float>(3, 1) << v[29][0]*1000,v[29][1]*1000,v[29][2]*1000);
//     Mat robot_tr_31 = (Mat_<float>(3, 1) << v[30][0]*1000,v[30][1]*1000,v[30][2]*1000);
//     Mat robot_tr_32 = (Mat_<float>(3, 1) << v[31][0]*1000,v[31][1]*1000,v[31][2]*1000);
//     Mat robot_tr_33 = (Mat_<float>(3, 1) << v[32][0]*1000,v[32][1]*1000,v[32][2]*1000);
//     Mat robot_tr_34 = (Mat_<float>(3, 1) << v[33][0]*1000,v[33][1]*1000,v[33][2]*1000);
//     Mat robot_tr_35 = (Mat_<float>(3, 1) << v[34][0]*1000,v[34][1]*1000,v[34][2]*1000);
//     Mat robot_tr_36 = (Mat_<float>(3, 1) << v[35][0]*1000,v[35][1]*1000,v[35][2]*1000);
//     Mat robot_tr_37 = (Mat_<float>(3, 1) << v[36][0]*1000,v[36][1]*1000,v[36][2]*1000);
//     Mat robot_tr_38 = (Mat_<float>(3, 1) << v[37][0]*1000,v[37][1]*1000,v[37][2]*1000);
//     Mat robot_tr_39 = (Mat_<float>(3, 1) << v[38][0]*1000,v[38][1]*1000,v[38][2]*1000);
//     Mat robot_tr_40 = (Mat_<float>(3, 1) << v[39][0]*1000,v[39][1]*1000,v[39][2]*1000);

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
//     cout<< "Translation 11" <<endl << robot_tr_11<< endl << endl;
//     cout<< "Translation 12" <<endl << robot_tr_12<< endl << endl;
//     cout<< "Translation 13" <<endl << robot_tr_13<< endl << endl;
//     cout<< "Translation 14" <<endl << robot_tr_14<< endl << endl;
//     cout<< "Translation 15" <<endl << robot_tr_15<< endl << endl;
//     cout<< "Translation 16" <<endl << robot_tr_16<< endl << endl;
//     cout<< "Translation 17" <<endl << robot_tr_17<< endl << endl;
//     cout<< "Translation 18" <<endl << robot_tr_18<< endl << endl;
//     cout<< "Translation 19" <<endl << robot_tr_19<< endl << endl;
//     cout<< "Translation 20" <<endl << robot_tr_20<< endl << endl;
//     cout<< "Translation 21" <<endl << robot_tr_21<< endl << endl;
//     cout<< "Translation 22" <<endl << robot_tr_22<< endl << endl;
//     cout<< "Translation 23" <<endl << robot_tr_23<< endl << endl;
//     cout<< "Translation 24" <<endl << robot_tr_24<< endl << endl;
//     cout<< "Translation 25" <<endl << robot_tr_25<< endl << endl;
//     cout<< "Translation 26" <<endl << robot_tr_26<< endl << endl;
//     cout<< "Translation 27" <<endl << robot_tr_27<< endl << endl;
//     cout<< "Translation 28" <<endl << robot_tr_28<< endl << endl;
//     cout<< "Translation 29" <<endl << robot_tr_29<< endl << endl;
//     cout<< "Translation 30" <<endl << robot_tr_30<< endl << endl;
//     cout<< "Translation 31" <<endl << robot_tr_31<< endl << endl;
//     cout<< "Translation 32" <<endl << robot_tr_32<< endl << endl;
//     cout<< "Translation 33" <<endl << robot_tr_33<< endl << endl;
//     cout<< "Translation 34" <<endl << robot_tr_34<< endl << endl;
//     cout<< "Translation 35" <<endl << robot_tr_35<< endl << endl;
//     cout<< "Translation 36" <<endl << robot_tr_36<< endl << endl;
//     cout<< "Translation 37" <<endl << robot_tr_37<< endl << endl;
//     cout<< "Translation 38" <<endl << robot_tr_38<< endl << endl;
//     cout<< "Translation 39" <<endl << robot_tr_39<< endl << endl;
//     cout<< "Translation 40" <<endl << robot_tr_40<< endl << endl;


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
//    R_gripper2base.push_back(robot_rot_11);
//    R_gripper2base.push_back(robot_rot_12);
//    R_gripper2base.push_back(robot_rot_13);
//    R_gripper2base.push_back(robot_rot_14);
//    R_gripper2base.push_back(robot_rot_15);
//    R_gripper2base.push_back(robot_rot_16);
//    R_gripper2base.push_back(robot_rot_17);
//    R_gripper2base.push_back(robot_rot_18);
//    R_gripper2base.push_back(robot_rot_19);
//    R_gripper2base.push_back(robot_rot_20);
//    R_gripper2base.push_back(robot_rot_21);
//    R_gripper2base.push_back(robot_rot_22);
//    R_gripper2base.push_back(robot_rot_23);
//    R_gripper2base.push_back(robot_rot_24);
//    R_gripper2base.push_back(robot_rot_25);
//    R_gripper2base.push_back(robot_rot_26);
//    R_gripper2base.push_back(robot_rot_27);
//    R_gripper2base.push_back(robot_rot_28);
//    R_gripper2base.push_back(robot_rot_29);
//    R_gripper2base.push_back(robot_rot_30);
//    R_gripper2base.push_back(robot_rot_31);
//    R_gripper2base.push_back(robot_rot_32);
//    R_gripper2base.push_back(robot_rot_33);
//    R_gripper2base.push_back(robot_rot_34);
//    R_gripper2base.push_back(robot_rot_35);
//    R_gripper2base.push_back(robot_rot_36);
//    R_gripper2base.push_back(robot_rot_37);
//    R_gripper2base.push_back(robot_rot_38);
//    R_gripper2base.push_back(robot_rot_39);
//    R_gripper2base.push_back(robot_rot_40);


    // with euler
//    R_gripper2base_Euler.push_back(robotE_rot_01);
//    R_gripper2base_Euler.push_back(robotE_rot_02);
//    R_gripper2base_Euler.push_back(robotE_rot_03);
//    R_gripper2base_Euler.push_back(robotE_rot_04);
//    R_gripper2base_Euler.push_back(robotE_rot_05);
//    R_gripper2base_Euler.push_back(robotE_rot_06);
//    R_gripper2base_Euler.push_back(robotE_rot_07);
//    R_gripper2base_Euler.push_back(robotE_rot_08);
//    R_gripper2base_Euler.push_back(robotE_rot_09);
//    R_gripper2base_Euler.push_back(robotE_rot_10);
//    R_gripper2base_Euler.push_back(robotE_rot_11);
//    R_gripper2base_Euler.push_back(robotE_rot_12);
//    R_gripper2base_Euler.push_back(robotE_rot_13);
//    R_gripper2base_Euler.push_back(robotE_rot_14);
//    R_gripper2base_Euler.push_back(robotE_rot_15);
//    R_gripper2base_Euler.push_back(robotE_rot_16);
//    R_gripper2base_Euler.push_back(robotE_rot_17);
//    R_gripper2base_Euler.push_back(robotE_rot_18);


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
//    t_gripper2base.push_back(robot_tr_11);
//    t_gripper2base.push_back(robot_tr_12);
//    t_gripper2base.push_back(robot_tr_13);
//    t_gripper2base.push_back(robot_tr_14);
//    t_gripper2base.push_back(robot_tr_15);
//    t_gripper2base.push_back(robot_tr_16);
//    t_gripper2base.push_back(robot_tr_17);
//    t_gripper2base.push_back(robot_tr_18);
//    t_gripper2base.push_back(robot_tr_19);
//    t_gripper2base.push_back(robot_tr_20);
//    t_gripper2base.push_back(robot_tr_21);
//    t_gripper2base.push_back(robot_tr_22);
//    t_gripper2base.push_back(robot_tr_23);
//    t_gripper2base.push_back(robot_tr_24);
//    t_gripper2base.push_back(robot_tr_25);
//    t_gripper2base.push_back(robot_tr_26);
//    t_gripper2base.push_back(robot_tr_27);
//    t_gripper2base.push_back(robot_tr_28);
//    t_gripper2base.push_back(robot_tr_29);
//    t_gripper2base.push_back(robot_tr_30);
//    t_gripper2base.push_back(robot_tr_31);
//    t_gripper2base.push_back(robot_tr_32);
//    t_gripper2base.push_back(robot_tr_33);
//    t_gripper2base.push_back(robot_tr_34);
//    t_gripper2base.push_back(robot_tr_35);
//    t_gripper2base.push_back(robot_tr_36);
//    t_gripper2base.push_back(robot_tr_37);
//    t_gripper2base.push_back(robot_tr_38);
//    t_gripper2base.push_back(robot_tr_39);
//    t_gripper2base.push_back(robot_tr_40);

    cout << R_gripper2base.size() << " R_gripper2base"<<endl;
    cout << t_gripper2base.size() << " t_gripper2base"<<endl;
    cout << R_target2cam.size() << " R_target2cam"<<endl;
    cout << R_target2cam.size() << " t_target2cam"<<endl;

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


//    //***************************** EULER*************************************
//    calibrateHandEye(R_gripper2base_Euler, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_Euler_TSAI, t_cam2gripper_Euler_TSAI, CALIB_HAND_EYE_TSAI);
//    calibrateHandEye(R_gripper2base_Euler, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_Euler_HORAUD, t_cam2gripper_Euler_HORAUD, CALIB_HAND_EYE_HORAUD);
//    calibrateHandEye(R_gripper2base_Euler, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_Euler_ANDREFF, t_cam2gripper_Euler_ANDREFF, CALIB_HAND_EYE_ANDREFF);
//    calibrateHandEye(R_gripper2base_Euler, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_Euler_PARK, t_cam2gripper_Euler_PARK, CALIB_HAND_EYE_PARK);
//    calibrateHandEye(R_gripper2base_Euler, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper_Euler_DAN, t_cam2gripper_Euler_DAN, CALIB_HAND_EYE_DANIILIDIS);
//    Vec3f R_cam2gripper_Euler_rT = rotationMatrixToEulerAngles(R_cam2gripper_Euler_TSAI);
//    Vec3f R_cam2gripper_Euler_rH = rotationMatrixToEulerAngles(R_cam2gripper_Euler_HORAUD);
//    Vec3f R_cam2gripper_Euler_rA = rotationMatrixToEulerAngles(R_cam2gripper_Euler_ANDREFF);
//    Vec3f R_cam2gripper_Euler_rP = rotationMatrixToEulerAngles(R_cam2gripper_Euler_PARK);
//    Vec3f R_cam2gripper_Euler_rD = rotationMatrixToEulerAngles(R_cam2gripper_Euler_DAN);


//    cv::Mat Rodrigues_Tsai_Euler, Rodrigues_Horaud_Euler, Rodrigues_Park_Euler, Rodrigues_Andreff_Euler, Rodrigues_Dan_Euler;
//    cv::Rodrigues(R_cam2gripper_Euler_TSAI,Rodrigues_Tsai_Euler);
//    cv::Rodrigues(R_cam2gripper_Euler_HORAUD,Rodrigues_Horaud_Euler);
//    cv::Rodrigues(R_cam2gripper_Euler_PARK,Rodrigues_Park_Euler);
//    cv::Rodrigues(R_cam2gripper_Euler_ANDREFF,Rodrigues_Andreff_Euler);
//    cv::Rodrigues(R_cam2gripper_Euler_DAN,Rodrigues_Dan_Euler);


//    // Rotation
//    cout << "R_cam2gripper_Euler_TSAI = " << endl << " " << R_cam2gripper_Euler_TSAI << endl << endl;
//    cout << "R_cam2gripper_Euler_rT = " << endl << " " << R_cam2gripper_Euler_rT << endl << endl;
//    cout << " Rodrigues R_cam2gripper_rT_Euler = " << endl << " " << Rad2degV(Rodrigues_Tsai_Euler) << endl << endl;

//    cout << "R_cam2gripper_Euler_HORAUD = " << endl << " " << R_cam2gripper_Euler_HORAUD << endl << endl;
//    cout << "R_cam2gripper_Euler_rH = " << endl << " " << R_cam2gripper_Euler_rH << endl << endl;
//    cout << " Rodrigues R_cam2gripper_rH_Euler = " << endl << " " << Rad2degV(Rodrigues_Horaud_Euler) << endl << endl;

//    cout << "R_cam2gripper_Euler_PARK = " << endl << " " << R_cam2gripper_Euler_PARK << endl << endl;
//    cout << "R_cam2gripper_Euler_rP = " << endl << " " << R_cam2gripper_Euler_rP << endl << endl;
//    cout << " Rodrigues R_cam2gripper_rP_Euler = " << endl << " " << Rad2degV(Rodrigues_Park_Euler) << endl << endl;

//    cout << "R_cam2gripper_Euler_ANDREFF = " << endl << " " << R_cam2gripper_Euler_ANDREFF << endl << endl;
//    cout << "R_cam2gripper_Euler_rA = " << endl << " " << R_cam2gripper_Euler_rA << endl << endl;
//    cout << " Rodrigues R_cam2gripper_rA_Euler = " << endl << " " << Rad2degV(Rodrigues_Andreff_Euler) << endl << endl;

//    cout << "R_cam2gripper_Euler_DAN = " << endl << " " << R_cam2gripper_Euler_DAN << endl << endl;
//    cout << "R_cam2gripper_Euler_rD = " << endl << " " << R_cam2gripper_Euler_rD << endl << endl;
//    cout << " Rodrigues R_cam2gripper_rD_Euler = " << endl << " " << Rad2degV(Rodrigues_Dan_Euler) << endl << endl;

//    // Translation
//    cout << "t_cam2gripper_Euler_TSAI = " << endl << " " << t_cam2gripper_Euler_TSAI << endl << endl;
//    cout << "t_cam2gripper_Euler_HORAUD = " << endl << " " << t_cam2gripper_Euler_HORAUD << endl << endl;
//    cout << "t_cam2gripper_Euler_PARK = " << endl << " " << t_cam2gripper_Euler_PARK << endl << endl;
//    cout << "t_cam2gripper_Euler_ANDREFF = " << endl << " " << t_cam2gripper_Euler_ANDREFF << endl << endl;
//    cout << "t_cam2gripper_Euler_DAN = " << endl << " " << t_cam2gripper_Euler_DAN << endl << endl;

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

Mat TransposeVector(Vec3f &v)
{

    Mat RV = Mat_<float>(3,1);

    RV.at<float>(0,0)=v[0];
    RV.at<float>(0,1)=v[1];
    RV.at<float>(0,2)=v[2];

    return RV;
}

Vec3f Poly_Scale(Vec3f theta_rv)
{
float rv_len = sqrt(pow(theta_rv[0], 2) + pow(theta_rv[1], 2) + pow(theta_rv[2], 2));
float scale = 1 - 2 * CV_PI / rv_len;
theta_rv = theta_rv * scale;

return theta_rv;
}
