// HAND EYE CALIBRATION WORKING FOR CLOSED FORM SOLUTIONS ONLY A NEW CALIBRATION IS MISSING

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

    vector<String> fn;
    glob("/home/anders/Master/Hand-eye-Calibration/Test_program/build/poses_and_images/images/*.bmp", fn, false);

    vector<Mat> images;
    size_t num_images = fn.size(); //number of bmp files in images folder
    cout << "number of images"<< num_images<<endl;
    Size patternsize(5, 7); //number of centers
    std::vector<Point2f> centers; //this will be filled by the detected centers
    float cell_size = 30;
    std::vector<Point3f> obj_points;

    R_gripper2base.reserve(num_images);
    t_gripper2base.reserve(num_images);
    R_target2cam.reserve(num_images);
    t_target2cam.reserve(num_images);

    for (int i = 0; i < patternsize.height; ++i)
        for (int j = 0; j < patternsize.width; ++j)
            obj_points.push_back(Point3f(float(j*cell_size),
                                         float(i*cell_size), 0.f));
//cout <<"Objectpoints: " <<endl<<obj_points<<endl;
//cout <<"Objectpoints: " <<endl<<obj_points.size()<<endl;
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
            t_target2cam.push_back(tvec*1);
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
     Mat robot_tr_05 = (Mat_<float>(3, 1) << -1197.3, -69.263198852539063, 335.5);
     Mat robot_tr_06 = (Mat_<float>(3, 1) << -1076.7, -104.6, 323.4);
     Mat robot_tr_07 = (Mat_<float>(3, 1) << -1162.7, -110.8, 396.7);
     Mat robot_tr_08 = (Mat_<float>(3, 1) << -1073.8, -128.3, 438.5);
     Mat robot_tr_09 = (Mat_<float>(3, 1) << -1073.8, -128.3, 438.5);
     Mat robot_tr_10 = (Mat_<float>(3, 1) << -1171.5, -166.3, 630.0);

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

    //cout << "R_gripper2base = " << endl << " " << R_gripper2base[0] << endl << endl;
    //cout << "t_gripper2base = " << endl << " " << t_gripper2base[0] << endl << endl;



    cout << R_gripper2base.size() << " R_gripper2base"<<endl;
    cout << t_gripper2base.size() << " t_gripper2base"<<endl;
    cout << R_target2cam.size() << " R_target2cam"<<endl;
    cout << R_target2cam.size() << " t_target2cam"<<endl;
    //cout << R_cam2gripper.size() << " R_cam2gripper"<<endl;
    //cout << t_cam2gripper.size() << " t_cam2gripper"<<endl<<endl;

    cout << "Input to calibrateHandEyed: "<<endl;
    cout << "R_gripper2base = "<< endl <<" " << R_gripper2base[4] << endl<< endl;
    cout << "t_gripper2base = "<< endl <<" " << t_gripper2base[4] << endl<< endl;
    cout << "R_target2cam = "<< endl <<" " << R_target2cam[4] << endl<< endl;
    cout << "t_target2cam = "<< endl <<" " << t_target2cam[4] << endl<< endl;





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

    //Rotation
    cout << "R_cam2gripper_TSAI = " << endl << " " << R_cam2gripper_TSAI << endl << endl;
    cout << "R_cam2gripper_rT = " << endl << " " << R_cam2gripper_rT << endl << endl;

    cout << "R_cam2gripper_HORAUD = " << endl << " " << R_cam2gripper_HORAUD << endl << endl;
    cout << "R_cam2gripper_rH = " << endl << " " << R_cam2gripper_rH << endl << endl;

    cout << "R_cam2gripper_ANDREFF = " << endl << " " << R_cam2gripper_ANDREFF << endl << endl;
    cout << "R_cam2gripper_rA = " << endl << " " << R_cam2gripper_rA << endl << endl;

    cout << "R_cam2gripper_PARK = " << endl << " " << R_cam2gripper_PARK << endl << endl;
    cout << "R_cam2gripper_rP = " << endl << " " << R_cam2gripper_rP << endl << endl;

    cout << "R_cam2gripper_DAN = " << endl << " " << R_cam2gripper_DAN << endl << endl;
    cout << "R_cam2gripper_rD = " << endl << " " << R_cam2gripper_rD << endl << endl;

    // Translation
    cout << "t_cam2gripper_TSAI = " << endl << " " << t_cam2gripper_TSAI << endl << endl;
    cout << "t_cam2gripper_HORAUD = " << endl << " " << t_cam2gripper_HORAUD << endl << endl;
    cout << "t_cam2gripper_PARK = " << endl << " " << t_cam2gripper_PARK << endl << endl;
    cout << "t_cam2gripper_ANDREFF = " << endl << " " << t_cam2gripper_ANDREFF << endl << endl;
    cout << "t_cam2gripper_DAN = " << endl << " " << t_cam2gripper_DAN << endl << endl;
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
