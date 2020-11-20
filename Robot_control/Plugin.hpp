#ifndef PLUGIN_HPP
#define PLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>

// RobWorkStudio includes
#include <RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

// UR RTDE includes
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

// RobWork library includes
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

// Boost includes
#include <boost/bind.hpp>

// Qt includes
#include <QPushButton>
#include <QGridLayout>

// Standard includes
#include <iostream>
#include <thread>
#include <utility>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <stdio.h>

// OpenCV
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

// Real sense
#include <librealsense2/rs.hpp>



// Extra defines for robot gripper
#define OPEN true
#define CLOSE false

class QPushButton;

class Plugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "Plugin.json")
public:
    Plugin();
    virtual ~Plugin();

    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    // Plugin default
    void clickEvent();
    void stateChangedListener(const rw::kinematics::State& state);

    // Functions
    void RunRobotMimic();
    void RunRobotControl();
    void RunHomeRobot();
    void printQ();
    void printTCP();
    void MoveInToolSpace();


    // Start thread
    void startRobotMimic();
    void startRobotControl();
    void startHomeRobot();

    // Manage
    void connectRobot();
    void stopRobot();
    void teachModeToggle();


    // Assistive
    void printArray(std::vector<double>);
    std::vector<double> addMove(std::vector<double>, double, double);
    void write_vector_to_file(const std::vector<double>& myVector, std::string filename);

    // Planning
    void createPathRRTConnect(std::vector<double>, std::vector<double>, double, std::vector<std::vector<double>>&, rw::kinematics::State);

    // Camera API
    void Take_picture();
    void Analyze_images();

private:
    // Qt buttons
    QPushButton *_btn0,*_btn1,*_btn2,*_btn3,*_btn4, *_btn5, *_btn6, *_btn7, *_btn8, *_btn9, *_btn10;

    // RobWorkStudio interface
    rw::proximity::CollisionDetector::Ptr collisionDetector;
    rw::models::WorkCell::Ptr rws_wc;
    rw::kinematics::State rws_state;
    rw::models::Device::Ptr rws_robot;

    // UR interface
    std::string ur_robot_ip = "192.168.1.210";
    ur_rtde::RTDEControlInterface   *ur_robot;
    ur_rtde::RTDEIOInterface        *ur_robot_io;
    ur_rtde::RTDEReceiveInterface   *ur_robot_receive;

    // Positions                       j0        j1        j2        j3        j4        j5
    std::vector<double> gripQ =     {  1.03566, -1.18752,  1.98773, -2.39819, -1.55003, -1.74102 };
    std::vector<double> gripTCP =   { -0.15573, -0.52874,  0.17813,  1.77626, -2.57197,  0.04202 };
    //std::vector<double> homeQ =     {  1.17810, -1.57080,  1.57080, -1.57080, -1.57080, -1.57080 };
    std::vector<double> homeTCP =   { -0.06489, -0.50552,  0.48784, -1.74588,  2.61176,  0.00493 };
    std::vector<double> homeQ =     {  1.777145, -1.58239, 1.58379, -3.20074, -1.61651, 0.0158763 };

    // Positions                      X           Y       Z          Rx         RY        RZ
    std::vector<double> MoveHome =  { 0.0417281, -0.7792, 0.0548525, 0.847933, -2.26498, -0.279166 };
    std::vector<double> MoveX =     { 0.0500000, -0.7792, 0.0548525, 0.847933, -2.26498, -0.279166 };
    std::vector<double> MoveZ =     { 0.0500000, -0.7792, 0.0800000, 0.847933, -2.26498, -0.279166 };
    std::vector<double> MoveY =     { 0.0500000, -0.9000, 0.0800000, 0.847933, -2.26498, -0.279166 };
    std::vector<double> Crazy =     { 0.0000000, 0.00000, 0.5000000, 0.847933, -2.26498, -0.279166 };

    // Flags
    std::atomic_bool ur_robot_exists;
    std::atomic_bool ur_robot_stopped;
    std::atomic_bool ur_robot_teach_mode;

    // Threads
    std::thread control_thread;
    std::thread update_thread;
    std::thread home_thread;
};

#endif /*PLUGIN_HPP*/
