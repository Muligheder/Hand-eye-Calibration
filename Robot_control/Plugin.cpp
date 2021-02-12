#include "Plugin.hpp"
    #include "/home/anders/librealsense/examples/example.hpp" // Include short list of convenience functions for rendering
Plugin::Plugin():
    rws::RobWorkStudioPlugin("Plugin", QIcon(":/plugin.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    // Define button layout
    int row = 0;

    // Initiation
    QLabel *label_init = new QLabel(this);
    label_init->setText("Initiation");
    pLayout->addWidget(label_init,row++,0);



    _btn0 = new QPushButton("Connect");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Synchronize movement");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_stop_sync = new QPushButton("Stop synchronized movement");
    pLayout->addWidget(_btn_stop_sync, row++, 0);
    connect(_btn_stop_sync, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn2 = new QPushButton("Start robot control");
    pLayout->addWidget(_btn2, row++, 0);
    connect(_btn2, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn3 = new QPushButton("Stop robot");
    pLayout->addWidget(_btn3, row++, 0);
    connect(_btn3, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn4 = new QPushButton("Toggle teach mode");
    pLayout->addWidget(_btn4, row++, 0);
    connect(_btn4, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn5 = new QPushButton("Home robot");
    pLayout->addWidget(_btn5, row++, 0);
    connect(_btn5, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn6 = new QPushButton("Print state");
    pLayout->addWidget(_btn6, row++, 0);
    connect(_btn6, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn7 = new QPushButton("Print TCP");
    pLayout->addWidget(_btn7, row++, 0);
    connect(_btn7, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn8 = new QPushButton("Move In X");
    pLayout->addWidget(_btn8, row++, 0);
    connect(_btn8, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn9 = new QPushButton("Take Pictures");
    pLayout->addWidget(_btn9, row++, 0);
    connect(_btn9, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn10 = new QPushButton("Extrating Camera T-Matrix");
    pLayout->addWidget(_btn10, row++, 0);
    connect(_btn10, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn11 = new QPushButton("Extrating Robot T-Matrix");
    pLayout->addWidget(_btn11, row++, 0);
    connect(_btn11, SIGNAL(clicked()), this, SLOT(clickEvent()));


    pLayout->setRowStretch(row,1);

    // Initialize flags
    ur_robot_exists = false;
    ur_robot_teach_mode = false;
    ur_robot_stopped = true;
    rws_robot_synced = false;
}

Plugin::~Plugin()
{
}

void Plugin::initialize()
{
    std::cout << "Start of initialize()" << std::endl;
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&Plugin::stateChangedListener, this, boost::arg<1>()), this);
    std::cout << "End of initialize()" << std::endl;
}

void Plugin::open(rw::models::WorkCell* workcell)
{
    std::cout << "Start of open()" << std::endl;

    // If workcell exists
    if (workcell != NULL)
    {
        // print device info
        printDeviceNames(*workcell);
        // Get rws info
        rws_wc = workcell;
        //rw::kinematics::State s = rws_state = rws_wc->getDefaultState();
        rws_state       = rws_wc->getDefaultState();
        //rw::models::SerialDevice::Ptr rws_robot = rws_wc->findDevice<rw::models::SerialDevice>("UR5e_2018");
        rws_robot       = rws_wc->findDevice<rw::models::SerialDevice>("UR5e_2018");
        rws_robot_base  = rws_wc->findFrame<rw::kinematics::Frame>("UR5e_2018.Base");
        rws_table       = rws_wc->findFrame<rw::kinematics::Frame>("Table");

        if(rws_robot == NULL)
        {
                std::cout << "Couldn't locate rws_robot!" << std::endl;
                return;
        }
        // Use rws collision checker
        collisionDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(rws_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    }

    std::cout << "End of open()" << std::endl;
}

void Plugin::close()
{
    std::cout << "Start of close()" << std::endl;
    std::cout << "End of close()" << std::endl;
}

void Plugin::clickEvent()
{
    // log().info() << "Button 0 pressed!\n";
    QObject *obj = sender();

    if(obj == _btn0)
        connectRobot();
    else if(obj == _btn1)
        startRobotMimic();
    else if(obj == _btn2)
        startRobotControl();
    else if(obj == _btn3)
        stopRobot();
    else if(obj == _btn4)
        teachModeToggle();
    else if(obj == _btn5)
        startHomeRobot();
    else if(obj == _btn6)
        printQ();
    else if(obj == _btn7)
        printTCP();
    else if(obj == _btn8)
        MoveInToolSpace();
    else if(obj == _btn9)
        Take_picture();
    else if(obj == _btn10)
        Analyze_images();
    else if(obj == _btn11)
        TransformationRobot();
    else if(obj == _btn_stop_sync)
        stopSync();

}

void Plugin::stateChangedListener(const rw::kinematics::State& state)
{
    rws_state = state;
    log().info() << "State changed!";
}

void Plugin::connectRobot()
{
    std::cout << "Connecting to " << ur_robot_ip << "..." << std::endl;
    if(!ur_robot_exists)
    {
        std::cout << "Control interface:\t";
        ur_robot = new ur_rtde::RTDEControlInterface(ur_robot_ip);
        std::cout << "Connected!" << std::endl;
        std::cout << "Receive interface:\t";
        ur_robot_receive = new ur_rtde::RTDEReceiveInterface(ur_robot_ip);
        std::cout << "Connected!" << std::endl;
        std::cout << "IO interface:\t";
        ur_robot_io = new ur_rtde::RTDEIOInterface(ur_robot_ip);
        std::cout << "Connected!" << std::endl;

        ur_robot_exists = true;
        std::cout << "Robot ready!" << std::endl;
    }
    else
        std::cout << "Already connected..." << std::endl;
}

void Plugin::printDeviceNames(const rw::models::WorkCell& workcell)
{
    std::cout << "Workcell " << workcell << " contains devices:" << std::endl;
    for(const rw::models::Device::CPtr device : workcell.getDevices()) {
        std::cout << "- " << device->getName() << std::endl;
    }
}




std::vector<double> Plugin::addMove(std::vector<double> position, double acceleration = 0.5, double velocity = 0.5, double blend = 0.2)
{
    std::vector<double> move = { acceleration, velocity };
    std::vector<double> position_and_move;
    position_and_move.reserve(position.size() + move.size());
    position_and_move.insert( position_and_move.end(), position.begin(), position.end() );
    position_and_move.insert( position_and_move.end(), move.begin(), move.end() );
    return position_and_move;
}




void Plugin::RunRobotControl()
{
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    if(ur_robot_teach_mode)
    {
        std::cout << "Teach mode enabled..." << std::endl;
        return;
    }

    std::vector<double> grip = addMove(gripQ, 0.2, 0.2);
    std::vector<double> home = addMove(homeQ);

    std::vector<std::vector<double>> path;

    ur_robot_stopped = false;
    while(!ur_robot_stopped)
    {
        // Open and move down
        path.clear();
        ur_robot_io->setStandardDigitalOut(0,OPEN);
        path.push_back(grip);
        ur_robot->moveJ(path);

        // Close and move up
        ur_robot_io->setStandardDigitalOut(0,CLOSE);
        path.clear();
        path.push_back(home);
        ur_robot->moveJ(path);
    }
    // ur_robot->stopScript(); // Stops further actions
}

void Plugin::teachModeToggle()
{
    if(ur_robot_exists)
    {
        if(ur_robot_teach_mode)
        {
            ur_robot->endTeachMode();
            std::cout << "Disabling teach mode!" << std::endl;
           // printArray(ur_robot_receive->getActualTCPPose());
            ur_robot_teach_mode = false;
        }
        else
        {
            ur_robot->teachMode();
            std::cout << "Enabling teach mode!" << std::endl;
           // printArray(ur_robot_receive->getActualTCPPose());
            ur_robot_teach_mode = true;

        }
    }
    else
        std::cout << "Robot not connected..." << std::endl;
}

void Plugin::RunRobotMimic()
{
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    rws_robot_synced = true;
    while(rws_robot_synced)
    {
        std::vector<double> currentQ = ur_robot_receive->getActualQ();
        rw::kinematics::State s = rws_state;
        rws_robot->setQ(currentQ, s);
        getRobWorkStudio()->setState(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Plugin::startRobotMimic()
{
    if(update_thread.joinable())
        update_thread.join();
    update_thread = std::thread(&Plugin::RunRobotMimic, this);
}

void Plugin::startRobotControl()
{
    if(control_thread.joinable())
        control_thread.join();
    control_thread = std::thread(&Plugin::RunRobotControl, this);
}

void Plugin::startHomeRobot()
{
    if(home_thread.joinable())
        home_thread.join();
    home_thread = std::thread(&Plugin::RunHomeRobot, this);
}

void Plugin::stopRobot()
{
    std::cout << "Stopping robot..." << std::endl;
    ur_robot->stopL();
    ur_robot->stopJ();
    ur_robot_stopped = true;
}

void Plugin::stopSync()
{
    std::cout << "Stopping robot sync..." << std::endl;
    rws_robot_synced = false;
}

void Plugin::RunHomeRobot()
{
    std::vector<std::vector<double>> route;
    //generatePythonRoute(route);
    printArray(route[0]);

    std::cout << "Homing robot..." << std::endl;
    rw::kinematics::State tmp_state = rws_state.clone();

    //std::vector<std::vector<double>> path;
    //std::vector<double> fromQ = ur_robot_receive->getActualQ();
    std::vector<double> fromQ = rws_robot->getQ(tmp_state).toStdVector();
    std::vector<double> toQ = invKin(fromQ, placeApproachL_RW);
    rws_robot->setQ(toQ, tmp_state);
    getRobWorkStudio()->setState(tmp_state);

    /*
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    if(ur_robot_teach_mode)
    {
        std::cout << "Teach mode enabled..." << std::endl;
        return;
    }


    std::cout << "Homing robot..." << std::endl;

    std::vector<std::vector<double>> path;
    std::vector<double> fromQ = ur_robot_receive->getActualQ();
    std::vector<double> toQ = homeQ;
    rw::kinematics::State tmp_state = rws_state;

    createPathRRTConnect(fromQ, toQ, 0.05, path, tmp_state);

    std::cout << "Moving robot..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ur_robot->moveJ(path);*/
}

void Plugin::MoveInToolSpace()
{
    /*
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    if(ur_robot_teach_mode)
    {
        std::cout << "Teach mode enabled..." << std::endl;
        return;
    }

    std::vector<double> home = addMove(MoveHome, 0.1, 0.1);
    std::vector<double> NewX = addMove(MoveX, 0.1, 0.1);
    std::vector<double> NewZ = addMove(MoveZ, 0.1, 0.1);
    std::vector<double> NewY = addMove(MoveY, 0.1, 0.1);
  //  std::vector<double> NewCrazy = addMove(Crazy, 0.1, 0.1);

    std::vector<std::vector<double>> path;

    path.clear();
    path.push_back(NewZ);
    ur_robot->moveL(path);
    write_vector_to_file(NewZ,"test.txt");

    path.clear();
    path.push_back(NewX);
    ur_robot->moveL(path);
    write_vector_to_file(NewX,"test.txt");

    path.clear();
    path.push_back(NewY);
    ur_robot->moveL(path);
    write_vector_to_file(NewY,"test.txt");

    path.clear();
    path.push_back(home);
    ur_robot->moveL(path);
    write_vector_to_file(home,"test.txt");

//    path.clear();
//    path.push_back(NewCrazy);
//    ur_robot->moveL(path);
//    write_vector_to_file(NewCrazy,"test.txt");*/

   // ur_rtde::RTDEControlInterface rtde_control("192.168.1.210");
    //ur_rtde::RTDEReceiveInterface rtde_receive("192.168.1.210");
    std::vector<double> init_q = ur_robot_receive->getActualQ();
    printArray(init_q);

    // Target in the robot base
    std::vector<double> new_q = init_q;
    new_q[0] += 0.2;
    printArray(new_q);
    /**
    * Move asynchronously in joint space to new_q, we specify asynchronous behavior by setting the async parameter to
    * 'true'. Try to set the async parameter to 'false' to observe a default synchronous movement, which cannot be
    * stopped by the stopJ function due to the blocking behaviour.
    */

    ur_robot->moveJ(new_q, 1.05, 1.4);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // Stop the movement before it reaches new_q
    ur_robot->stopJ(0.5);

    // Target 10 cm up in the Z-Axis of the TCP
    std::vector<double> target = ur_robot_receive->getActualTCPPose();
    printArray(target);
    target[2] += 0.10;
    printArray(target);
    /**
    * Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter
    * to 'true'. Try to set the async parameter to 'false' to observe a default synchronous movement, which cannot be
    * stopped by the stopL function due to the blocking behaviour.
    */

    ur_robot->moveL(target, 0.25, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // Stop the movement before it reaches target
    ur_robot->stopL(0.5);
    printArray(target);


    // Move to initial joint position with a regular moveJ
    ur_robot->moveJ(init_q);
    printArray(init_q);
    // Stop the RTDE control script
    //ur_robot->stopScript();
}

void Plugin::write_vector_to_file(const std::vector<double>& myVector, std::string filename)
{
    std::ofstream ofs(filename, std::ios::app | std::ofstream::binary);
    for(size_t i = 0; i < myVector.size()-1; i++)

        ofs << myVector[i] << " ";

    ofs << myVector[myVector.size()-1]<< std::endl;
}

void Plugin::printQ()
{
    std::cout << "Printing Q: " << std::endl;
    std::vector<double> const &input = ur_robot_receive->getActualQ();
    std::cout << "{ ";
    for(size_t i = 0; i < input.size()-1; i++)
        std::cout << input[i] << ", ";
    std::cout << input[input.size()-1] << " }" << std::endl;
}

void Plugin::printTCP()
{
    std::cout << "Printing TCP: " << std::endl;
    std::vector<double> const &input = ur_robot_receive->getActualTCPPose();
    std::cout << "{ ";
    for(size_t i = 0; i < input.size()-1; i++)
        std::cout << input[i] << ", ";
    std::cout << input[input.size()-1] << " }" << std::endl;
}

void Plugin::printArray(std::vector<double> input)
{
    std::cout << "{ ";
    for(size_t i = 0; i < input.size()-1; i++)
        std::cout << input[i] << ", ";
    std::cout << input[input.size()-1] << " }" << std::endl;
}

void Plugin::createPathRRTConnect(std::vector<double> from, std::vector<double> to, double epsilon, double velocity, double acceleration, double blend, std::vector<std::vector<double>> &path, rw::kinematics::State state)
{
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(collisionDetector.get(), rws_robot, state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(rws_robot), constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    rw::trajectory::QPath qpath;
    std::cout << "Generating path with NO max. time. Be patient or cancel manually... " << std::endl;
    planner->query(from, to, qpath); // DOES THIS JUST ACCEPT VECTORS OF DOUBLES???
    std::cout << "Found path of size " << qpath.size() << '!' << std::endl;

    path.clear();
    int index = 0;
    for(const auto &q : qpath)
    {
        std::vector<double> q_copy = q.toStdVector();
        if(index < qpath.size()-2)
        {
            path.push_back(addMove(q_copy, velocity, acceleration, blend));
            std::cout << "This is not last point" << std::endl;
        }
        else
        {
            path.push_back(addMove(q_copy, velocity, acceleration, 0));
            std::cout << "This is last point" << std::endl;
            //printArray(q.toStdVector());
        }
        index++;
    }
}

std::vector<double> Plugin::invKin(std::vector<double> startQ, std::vector<double> goalL)
{
    double theta = 22.5 * (M_PI / 180);

    // Duplicate state
    rw::kinematics::State tmp_state = rws_state.clone();

    // Create solver
    const rw::invkin::ClosedFormIKSolverUR solver(rws_robot, tmp_state);

    // Create goal transform object
    std::cout << "Goal (TCP):" << std::endl;
    printArray(goalL);

    const rw::math::Transform3D<> homeT = rws_robot_base->wTf(tmp_state);
    //const rw::math::Transform3D<> homeT = rws_table->wTf(tmp_state);

    const rw::math::Transform3D<> Tcorrection(
            rw::math::Vector3D<>(0, 0, 0),
            rw::math::RPY<>(-theta, 0, 0));

    const rw::math::Transform3D<> Tdesired(
            rw::math::Vector3D<>(goalL[0], goalL[1], goalL[2]),
            rw::math::RPY<>(goalL[3], goalL[4], goalL[5]));

    const rw::math::Transform3D<> Tdesired_wtf = Tcorrection*Tdesired;

    const std::vector<rw::math::Q> solutions = solver.solve(Tdesired_wtf, tmp_state);
    std::cout << "Found " << solutions.size() << " inverse kinematic solutions!" << std::endl;

    // Use shorted config distance
    int best_solution_index = 0;
    double best_solution_distance = 99999;
    int index = 0;
    for(const auto &solution : solutions)
    {
        rws_robot->setQ(solution, tmp_state);
        //getRobWorkStudio()->setState(tmp_state);
        if( !collisionDetector->inCollision(tmp_state,NULL,true) )
        {
            double solution_distance = getConfDistance(startQ, solution.toStdVector());
            if(solution_distance < best_solution_distance)
            {
                best_solution_index = index;
                best_solution_distance = solution_distance;
            }
        }
        index++;
    }

    return solutions[best_solution_index].toStdVector();
}

double Plugin::getConfDistance(std::vector<double> a, std::vector<double> b)
{
    double sum = 0;
    for(size_t i = 0; i<a.size(); i++)
    {
        sum += pow((a[i] - b[i]),2);
    }
    sum = sqrt(sum);
    return sum;
}

void Plugin::Take_picture()
{


    // char tipka;
    char key;
    char filename[100]; // For filename
    int  c = 1; // For filename
    //rs2_pose pose;
    std::cout << "starting pipe.." << std::endl;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_BGR8, 60);

    // Start pipeline with chosen configuration
    pipe.start(cfg);
    std::cout << "capture frame..." << std::endl;

    rs2::frameset frames;
    //    for(int i = 0; i < 30; i++)
    //    {
    //        //Wait for all configured streams to produce a frame
    //        frames = pipe.wait_for_frames();
    //    }


    cv::Size patternsize(6, 9); //number of centers
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    double cell_size = 20;
    std::vector<cv::Point3f> obj_points;
    for (int i = 0; i < patternsize.height; ++i)
        for (int j = 0; j < patternsize.width; ++j)
            obj_points.push_back(cv::Point3f(double(j*cell_size),
                                             double(i*cell_size), 0.f));


    while(true)
    {
        // std::cerr << "lets begin.." << std::endl;
        frames = pipe.wait_for_frames();

        rs2::frame color_frame = frames.get_color_frame();

        // Creating OpenCV Matrix from a color image
        cv::Mat color(cv::Size(960, 540), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // save Image
        if(color.empty())
        {
            std::cerr << "Something is wrong with the camera, could not get frame." << std::endl;
            break;
        }
        usleep(5);
        cv::imshow("Lidar Camera L515", color);

        key =  cvWaitKey(25);

        if( key == '1')      //CAPTURING IMAGE FROM CAM
        {
            bool patternfound = cv::findChessboardCorners(color, patternsize, centers);
            if (patternfound)
            {
                sprintf(filename, "/home/anders/Hand-eye-Calibration/Robot_control/workcell/Images/0-%d.jpg", c);
                cv::imwrite(filename, color);
                cv::drawChessboardCorners(color, patternsize, cv::Mat(centers), patternfound);
                cv::imshow("CAMERA 1", color);    
                std::cout << "Cam 1 image captured   = " << c << std::endl;
                std::vector<double> fromQ = ur_robot_receive->getActualTCPPose();

                write_vector_to_file(fromQ,"test.txt");


                c++;
            }
            std::cout << patternfound << std::endl;

        }

        if(key == 27)
        {
            std::cout << "Escape Pressed\n";
            std::cerr << "destroy windows and ends video stream..." << std::endl;
            cv::destroyAllWindows();
            break;

        }


    }


}
void Plugin::TransformationRobot()
{
    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    if(!ur_robot_exists)
    {
        std::cout << "Robot not connected..." << std::endl;
        return;
    }

    if(ur_robot_teach_mode)
    {
        std::cout << "Teach mode enabled..." << std::endl;
        return;
    }
    std::vector<double> actualL=ur_robot_receive->getActualTCPPose();
    printArray(actualL);
    std::cout << " <-- TCP POSE" << std::endl;

    std::vector<double> RzRyRx(actualL.end() - 3, actualL.end());
    printArray(RzRyRx);
    std::cout << " <-- RzRyRx" << std::endl;

    cv::Vec3d theta;

    theta[0]=RzRyRx[0];
    theta[1]=RzRyRx[1];
    theta[2]=RzRyRx[2];
    std::cout << theta << std::endl;

    cv::Mat robot_rot = eulerAnglesToRotationMatrix(theta);
    cv::Mat robot_tr = (cv::Mat_<double>(3, 1)<< actualL[0],actualL[1],actualL[2]);

    t_gripper2base.push_back(robot_tr);
    R_gripper2base.push_back(robot_rot);
    std::cout << "R_gripper2base = " << std::endl << " " << R_gripper2base[0] << std::endl << std::endl;
    std::cout << "t_gripper2base = " << std::endl << " " << t_gripper2base[0] << std::endl << std::endl;
}



void Plugin::Analyze_images()
{
    // Camera calibration information

    // from example
    std::vector<double> distortionCoefficients(5);  // camera distortion
   /* distortionCoefficients[0] = 9.6349551984637724e-02;
    distortionCoefficients[1] = -3.3260675111130217e-01;
    distortionCoefficients[2] = 0;
    distortionCoefficients[3] = 0;
    distortionCoefficients[4] = 2.5833277679122602e-01;

    double f_x = 1.2993539019658076e+03; // Focal length in x axis
    double f_y = 1.2993539019658076e+03; // Focal length in y axis (usually the same?)
    double c_x = 960; // Camera primary point x
    double c_y = 540; // Camera primary point y
    */
    // My camera
    distortionCoefficients[0] = 1.83375015854836e-01;
    distortionCoefficients[1] = -5.65327823162079e-01;
    distortionCoefficients[2] = -1.45305093610659e-04;
    distortionCoefficients[3] = 5.0213176291436e-04;
    distortionCoefficients[4] = 5.08288383483887e-01;

    double f_x = 6.8233544921875e+02; // Focal length in x axis
    double f_y = 6.82471923828125e+02; // Focal length in y axis (usually the same?)
    double c_x = 960/2; // Camera primary point x
    double c_y = 540/2; // Camera primary point y

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

    cv::Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);
    //

    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    cv::Mat R_cam2gripper = (cv::Mat_<float>(3, 3));
    cv::Mat t_cam2gripper = (cv::Mat_<float>(3, 1));

    std::vector<std::string> fn;
    cv::glob("/home/anders/Hand-eye-Calibration/Test_program/build/poses_and_images/images/*.bmp", fn, false);

    std::vector<cv::Mat> images;
    size_t num_images = fn.size(); //number of bmp files in images folder
    std::cout << "number of images"<< num_images<<std::endl;
    cv::Size patternsize(6, 9); //number of centers
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    float cell_size = 20;
    std::vector<cv::Point3f> obj_points;

    R_gripper2base.reserve(num_images);
    t_gripper2base.reserve(num_images);
    R_target2cam.reserve(num_images);
    t_target2cam.reserve(num_images);

    for (int i = 0; i < patternsize.height; ++i)
        for (int j = 0; j < patternsize.width; ++j)
            obj_points.push_back(cv::Point3f(float(j*cell_size),
                                         float(i*cell_size), 0.f));
//cout <<"Objectpoints: " <<endl<<obj_points<<endl;
//cout <<"Objectpoints: " <<endl<<obj_points.size()<<endl;
    for (size_t i = 0; i < num_images; i++)
        images.push_back(cv::imread(fn[i]));


    cv::Mat frame;

    for (size_t i = 0; i < num_images; i++)
    {
        frame = cv::imread(fn[i]); //source image
        cv::Mat gray;
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

//          cv::imshow("window",gray);
//         cv::waitKey(0);

        bool patternfound = cv::findChessboardCorners(frame, patternsize, centers);
        if (patternfound)
        {
          //  cornerSubPix(gray, centers, Size(1, 1), Size(-1, -1),
            //                  TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            cv::drawChessboardCorners(frame, patternsize, cv::Mat(centers), patternfound);
          //   cv::imshow("window", frame);
          // int key = cv::waitKey(0) & 0xff;
            cv::solvePnP(cv::Mat(obj_points), cv::Mat(centers), cameraMatrix, distortionCoefficients, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);
           // cout <<"Rotation vetor = "<<endl<<" " <<rvec<<endl<<endl;


            cv::Mat R;
            cv::Rodrigues(rvec, R); // R is 3x3
            R_target2cam.push_back(R);
            t_target2cam.push_back(tvec*1);
            cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
            T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
            T(cv::Range(0, 3), cv::Range(3, 4)) = tvec * 1; // copies tvec into T

            std::cout << "T = " << std::endl << " " << T << std::endl << std::endl;

        }
        std::cout << patternfound << std::endl;
    }
}


cv::Mat Plugin::eulerAnglesToRotationMatrix(cv::Vec3d &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(theta[2]), -sin(theta[2]),
        0, sin(theta[2]), cos(theta[2])
        );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
        cos(theta[1]), 0, sin(theta[1]),
        0, 1, 0,
        -sin(theta[1]), 0, cos(theta[1])
        );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
        cos(theta[0]), -sin(theta[0]), 0,
        sin(theta[0]), cos(theta[0]), 0,
        0, 0, 1);

    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;

    return R;

}

double Plugin::rad2deg(double radian) {
    double pi = 3.14159;
    return(radian * (180 / pi));
}

double Plugin::deg2rad(double degree) {
    double pi = 3.14159;
    return(degree * (pi / 180));
}

// Checks if a matrix is a valid rotation matrix.
bool Plugin::isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles.
cv::Vec3d Plugin::rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
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
    return cv::Vec3d(rad2deg(x), rad2deg(y), rad2deg(z));


}

cv::Mat Plugin::ReverseVector(cv::Mat &v)
{

    cv::Mat RV = cv::Mat_<float>(3,1);

    RV.at<float>(0,0)=v.at<float>(0,2);
    RV.at<float>(0,1)=v.at<float>(0,1);
    RV.at<float>(0,2)=v.at<float>(0,0);

    return RV;
}

