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

    _btn8 = new QPushButton("Easy Home");
    pLayout->addWidget(_btn8, row++, 0);
    connect(_btn8, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn9 = new QPushButton("Take Pictures");
    pLayout->addWidget(_btn9, row++, 0);
    connect(_btn9, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn10 = new QPushButton("Extrating Camera T-Matrix");
    pLayout->addWidget(_btn10, row++, 0);
    connect(_btn10, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn11 = new QPushButton("PCL");
    pLayout->addWidget(_btn11, row++, 0);
    connect(_btn11, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_glob_align = new QPushButton("Global alignment");
    pLayout->addWidget(_btn_glob_align, row++, 0);
    connect(_btn_glob_align, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_local_align = new QPushButton("Local alignment");
    pLayout->addWidget(_btn_local_align, row++, 0);
    connect(_btn_local_align, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn_global_test = new QPushButton("Pose estimation test");
    pLayout->addWidget(_btn_global_test, row++, 0);
    connect(_btn_global_test, SIGNAL(clicked()), this, SLOT(clickEvent()));


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
        PCL();
    else if(obj == _btn_stop_sync)
        stopSync();
    else if(obj == _btn_glob_align)
        globalAlignment();
    else if(obj == _btn_local_align)
        localAlignment();
    else if(obj == _btn_global_test)
        for(int i = 0; i< 25; i++)
        {
        globalAlignment();
        localAlignment();
        }

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

void Plugin::import3DPoint(std::vector<double> &point)
{

        std::vector<std::vector<double>> v;
        std::ifstream in( "test1.txt" );
        std::string record;

        while ( std::getline( in, record ) )
        {
            std::istringstream is( record );
            std::vector<double> row( ( std::istream_iterator<double>( is ) ),
                                     std::istream_iterator<double>() );
            v.push_back( row );
        }
        for(auto && p : v){
          point.insert(point.end(), p.begin(), p.end());
        }
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


    std::vector<double> point;
    import3DPoint(point);
    printArray(point);

    // Camera to target
    std::vector<double> cTt = point;

    // target to camera
    std::vector<double> tTc  = Pose_inv(cTt);

    // Camera to Gripper from visp
    std::vector<double> gTc = {-0.008496624885,  0.01004966999,  0.1188209676,  -0.026702465,  -0.03330274707,  -2.335288144};
    // Camera to Gripper from openCV
    //std::vector<double> gTc = {-0.007333253944998697,  0.00941774561076744,  0.1150842076108506,  -0.0114279, -0.0131705, -2.33678};

    // Gripper to Camera
    std::vector<double> cTg = Pose_inv(gTc);

    // base to gripper getActualTCPPose()
    //std::vector<double> bTg = {0.50643, -0.136141, 0.643752, -2.84425, -1.22914, -0.0897504};
    std::vector<double> bTg = ur_robot_receive->getActualTCPPose();

    // gripper to base
    std::vector<double> gTb = Pose_inv(bTg);

    // base to camera bTt=bTg*cTg⁻1*cTt
    std::vector<double> bTc = ur_robot->poseTrans(bTg,gTc);

    // base to target
    std::vector<double> bTt = ur_robot->poseTrans(bTc,cTt);

    // Offset
    std::vector<double> bTt_offset = ur_robot->poseTrans(bTt,cTg);

    // RRT Between points
    std::vector<std::vector<double>> path;
    std::vector<double> fromQ = ur_robot_receive->getActualQ();
    std::vector<double> toQ = invKin(fromQ,bTt_offset);
    rw::kinematics::State tmp_state = rws_state.clone();
    createPathRRTConnect(fromQ, toQ, 0.05, 0.8, 0.8, 0.02, path, tmp_state);
    path.push_back(addMove(toQ, 0.4, 0.4, 0));

    std::cout << "Moving robot..." << std::endl;


    ur_robot->moveJ(path);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));


//    QMatrix4x4 tranform;
//    transform.translate();
//    transform.rotate(QQuarternioin::from);
//    QMatrix4x4 t = transoform * transform2;

//    QTransform

//    printArray(target);
//    std::cout << "Moving robot..." << std::endl;
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    ur_robot->moveL(target,0.5,0.5);
//    std::cout << "Location reached.." << std::endl;
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

void Plugin::moveToJ(std::vector<double> goal, double acceleration, double velocity)
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

    std::cout << "Moving to location" << std::endl;
    ur_robot->moveJ(goal, acceleration, velocity);
}

void Plugin::MoveInToolSpace()
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

    std::vector<double> bTg = {0.50643, -0.136141, 0.643752, -2.84425, -1.22914, -0.0897504};
    std::cout << "Moving to location" << std::endl;
    ur_robot->moveJ_IK(bTg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

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
    //double theta = 22.5 * (M_PI / 180);


    // Duplicate state
    rw::kinematics::State tmp_state = rws_state.clone();

    // Create solver
    const rw::invkin::ClosedFormIKSolverUR solver(rws_robot, tmp_state);

    // Create goal transform object
    std::cout << "Goal (TCP):" << std::endl;
    printArray(goalL);

    //const rw::math::Transform3D<> homeT = rws_robot_base->wTf(tmp_state);
    //const rw::math::Transform3D<> homeT = rws_table->wTf(tmp_state);

//    const rw::math::Transform3D<> Tcorrection(
//            rw::math::Vector3D<>(0, 0, 0),
//            rw::math::RPY<>(-theta, 0, 0));

    const rw::math::Transform3D<> Tdesired(
            rw::math::Vector3D<>(goalL[0], goalL[1], goalL[2]),
            rw::math::EAA<>(goalL[3], goalL[4], goalL[5]));

    //const rw::math::Transform3D<> Tdesired_wtf = Tcorrection*Tdesired;

    const std::vector<rw::math::Q> solutions = solver.solve(Tdesired, tmp_state);
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
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);

    // Start pipeline with chosen configuration
    pipe.start(cfg);
    std::cout << "capture frame..." << std::endl;

    rs2::frameset frames;
    //    for(int i = 0; i < 30; i++)
    //    {
    //        //Wait for all configured streams to produce a frame
    //        frames = pipe.wait_for_frames();
    //    }


    cv::Size patternsize(5, 8); //number of centers
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    double cell_size = 28.6666666666;
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
        cv::Mat color(cv::Size(1920, 1080), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // save Image
        if(color.empty())
        {
            std::cerr << "Something is wrong with the camera, could not get frame." << std::endl;
            break;
        }
        usleep(5);
        cv::namedWindow("L515", cv::WINDOW_NORMAL);
        cv::resizeWindow("L515", cv::Size(960,540));
        cv::imshow("L515", color);

        key =  cvWaitKey(25);

        if( key == '1')      //CAPTURING IMAGE FROM CAM
        {
//            rs2::frameset frames;
//                for(int i = 0; i < 30; i++)
//                {
//                    //Wait for all configured streams to produce a frame
//                    frames = pipe.wait_for_frames();
//                }

            bool patternfound = cv::findChessboardCorners(color, patternsize, centers);
            if (patternfound)
            {
                sprintf(filename, "/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/Images/%02d.bmp", c);
                cv::imwrite(filename, color);
                cv::drawChessboardCorners(color, patternsize, cv::Mat(centers), patternfound);

                cv::namedWindow("CAMERA 1", cv::WINDOW_NORMAL);
                cv::resizeWindow("CAMERA 1", cv::Size(960, 540));
                cv::imshow("CAMERA 1", color);
                std::cout << "Cam 1 image captured   = " << c << std::endl;
                std::vector<double> fromQ = ur_robot_receive->getActualTCPPose();
                printArray(fromQ);

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

std::vector<double> Plugin::Pose_inv(std::vector<double> v)
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

    cv::Rodrigues(R_Inverse,R_Inverse_result);
    //std::cout <<std::endl << "R_Inverse_result " << std::endl << R_Inverse_result << std::endl;

    // cv::Mat --> std::vector
    std::vector<double>t_inv(t_Inverse.begin<double>(), t_Inverse.end<double>());
    std::vector<double>R_inv(R_Inverse_result.begin<double>(), R_Inverse_result.end<double>());

//    print(t_inv);
//    print(R_inv);

    std::vector<double> pose(t_inv);
        pose.insert(pose.end(), R_inv.begin(), R_inv.end());
       // printArray(pose);
    return pose;

}

std::tuple<uint8_t, uint8_t, uint8_t> Plugin::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}

ptr_cloud Plugin::points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
//    cv::Mat colorr(cv::Size(1280, 720), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
//    namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
//    imshow("Display Image", colorr);

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
        current_color =get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);

    }

   return cloud;
}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void *viewer_void)
{
   std::ofstream myfile;
   myfile.open("test1.txt", std::ofstream::out | std::ofstream::trunc);
   std::cout << "Picking event active" << std::endl;
   std::vector<double> fromQ;
   if(event.getPointIndex() != -1)
   {
       float x, y, z;
       event.getPoint(x, y, z);
       fromQ={x,y,z,0,0,0};

       std::ofstream myfile("test1.txt", std::ios::app | std::ofstream::binary);
       for(size_t i = 0; i < fromQ.size()-1; i++)

           myfile << fromQ[i] << " ";

       myfile << fromQ[fromQ.size()-1]<< std::endl;

       std::cout << x << ", " << y << ", " << z << std::endl;
   }
}

void Plugin::PCL()
{
    char key;
//    // Create a simple OpenGL window for rendering:
//    window app(1280, 720, "RealSense Pointcloud Example");
//    // Construct an object to manage view state
//    glfw_state app_state;
//    // register callbacks to allow manipulation of the pointcloud
//    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    pipe.start();

    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");

    while (!viewer.wasStopped()) // Application still alive?
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

        ptr_cloud cloud = points_to_pcl(points, color);
        viewer.removeAllPointClouds();
        viewer.addPointCloud<pcl::PointXYZRGB> (cloud, "Sample cloud",0);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Sample cloud");
        viewer.initCameraParameters();
        viewer.spinOnce(100);


        // Upload the color frame to OpenGL
        //app_state.tex.upload(color);

        // Draw the pointcloud
       // draw_pointcloud(app.width(), app.height(), app_state, points);


}
    viewer.close();

    for (int i = 0; i < 30; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto color1 = frames.get_color_frame();
    if (!color1)
    {
        color1 = frames.get_infrared_frame();
    }

    // Save current point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //std::cout <<"I did it" <<std::endl;
    ptr_cloud cloud = points_to_pcl(points, color1);

    // filter everthing out with a distance further than 1m.
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.7);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    //Saving the point cloud
    pcl::io::savePCDFileASCII("cloud_test.pcd", *cloud_filtered);

    // Vizualize point cloud and generate point clicking event
    pcl::visualization::PCLVisualizer visualizer("PCL visualizer");
    visualizer.setBackgroundColor (0.251, 0.251, 0.251);// Floral white 1, 0.98, 0.94 | Misty Rose 1, 0.912, 0.9 |
    visualizer.addCoordinateSystem (1.0);
    visualizer.registerPointPickingCallback(pp_callback, (void*) &visualizer);
    visualizer.addPointCloud<pcl::PointXYZRGB> (cloud_filtered, "sample cloud",0);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    visualizer.initCameraParameters();
    visualizer.spin();
}

void Plugin::globalAlignment()
{
    std::cout << "Doing global alignment" << std::endl;
    pcl::PCLPointCloud2::Ptr object_in (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr scene_in (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr scene_cropped (new pcl::PCLPointCloud2 ());

    //Load pcl file created with get25DImage()
    if (pcl::io::loadPCDFile("m1.pcd", *object_in) == -1)
    {
      PCL_ERROR ("Couldn't read object pointcloud \n");
      return;
    }
    if (pcl::io::loadPCDFile("cloud_test.pcd", *scene_in) == -1)
    {
      PCL_ERROR ("Couldn't read scene pointcloud \n");
      return;
    }
    Eigen::Vector4f minPoint = Eigen::Vector4f(-2, -2, -1, 1.0);
    Eigen::Vector4f maxPoint = Eigen::Vector4f(2, 2, 1, 1.0);

    cropScene(scene_in, scene_cropped, minPoint, maxPoint);

    pcl::PointCloud<pcl::PointNormal>::Ptr object_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // Downsample using a VoxelGrid filter
    std::cout << "Downsampling pointclouds" << std::endl;
    voxelGrid(object_in, object_filtered);
    voxelGrid(scene_cropped, scene_filtered);

    for(size_t i = 0; i < object_filtered->points.size(); ++i)
     {
          object_filtered->points[i].x *= 0.1;
          object_filtered->points[i].y *= 0.1;
          object_filtered->points[i].z *= 0.1;
    }

    std::cout << "Object size " << object_filtered->points.size()<< std::endl;
    std::cout << "Scene size " << scene_filtered->points.size()<< std::endl;
    //visualizePointClouds(scene_filtered, object_filtered, "Before global alignment");


    // Compute surface normals
    std::cout << "Computing surface normals" << std::endl;
    computeSurfaceNormals(object_filtered);
    computeSurfaceNormals(scene_filtered);

    // Compute shape features
    std::cout << "Computing shape features" << std::endl;
    pcl::PointCloud<pcl::Histogram<153>>::Ptr object_features(new pcl::PointCloud<pcl::Histogram<153>>);
    pcl::PointCloud<pcl::Histogram<153>>::Ptr scene_features(new pcl::PointCloud<pcl::Histogram<153>>);
    computeShapeFeatures(object_filtered, object_features);
    computeShapeFeatures(scene_filtered, scene_features);

    // Find feature matches
    std::cout << "Finding feature matches" << std::endl;
    pcl::Correspondences corr(object_features->size());
    {
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }

    // RANSAC parameters
    const size_t iter = 5000;
    const float threshsq = 0.01 * 0.01;

    auto [pose, object_aligned, glob_inliers, glob_rmse] = RANSAC(object_filtered, corr, iter, threshsq);

    glob_pose = pose;
    glob_object_align = object_aligned;

    // Set to true to display pointclouds
    bool visualize = false;

    if (visualize) {
        visualizePointClouds(scene_filtered, object_aligned, "After global alignment");
    }

    std::cout << "Finished global alignment!" << std::endl;
    std::cout << "Global transform is:\n" << glob_pose << std::endl;
    std::cout << "Rmse GLOBAL " << glob_rmse << std::endl;
    std::string filename = "Global_RMSE.txt";
    std::ofstream ofs(filename, std::ios::app | std::ofstream::binary);
    ofs << glob_rmse;
}

std::tuple<Eigen::Matrix4f, pcl::PointCloud<pcl::PointNormal>::Ptr, size_t, float> Plugin::RANSAC(pcl::PointCloud<pcl::PointNormal>::Ptr & object, pcl::Correspondences corr, const size_t iter, const float threshsq)
{
    // Create a k-d tree for scene
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(scene_filtered);

    std::cout << "Running RANSAC for " << iter << " iterations." << std::endl;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned(new pcl::PointCloud<pcl::PointNormal>);
    float penalty = FLT_MAX;

    pcl::common::UniformGenerator<int> gen(0, corr.size() - 1);
    for(size_t i = 0; i < iter; ++i) {

        // Sample 3 random correspondences
        std::vector<int> idxobj(3);
        std::vector<int> idxscn(3);
        for(int j = 0; j < 3; ++j) {
            const int idx = gen.run();
            idxobj[j] = corr[idx].index_query;
            idxscn[j] = corr[idx].index_match;
        }

        // Estimate transformation
        Eigen::Matrix4f T;
        pcl::registration::TransformationEstimationSVD<pcl::PointNormal,pcl::PointNormal> est;
        est.estimateRigidTransformation(*object, idxobj, *scene_filtered, idxscn, T);

        // Apply pose
        pcl::transformPointCloud(*object, *object_aligned, T);

        // Validate
        std::vector<std::vector<int> > idx;
        std::vector<std::vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

        // Compute inliers and RMSE
        size_t inliers = 0;
        float rmse = 0;
        for(size_t j = 0; j < distsq.size(); ++j)
            if(distsq[j][0] <= threshsq) {
                ++inliers;
                rmse += distsq[j][0];
            }
        rmse = sqrtf(rmse / inliers);

        // Evaluate a penalty function
        const float outlier_rate = 1.0f - float(inliers) / object->size();
        //const float penaltyi = rmse;
        const float penaltyi = outlier_rate;

        // Update result
        if(penaltyi < penalty) {
            std::cout << "Got a new model with " << inliers << " inliers after " << i << " iterations!\n" << std::endl;
            penalty = penaltyi;
            pose = T;

            std::cout << "Rmse " << rmse << std::endl;
        }
    }

    transformPointCloud(*object, *object_aligned, pose);

    // Compute inliers and RMSE
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;
    tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
    size_t inliers = 0;
    float rmse = 0;
    for(size_t i = 0; i < distsq.size(); ++i)
        if(distsq[i][0] <= threshsq) {
            ++inliers;
            rmse += distsq[i][0];
        }
    rmse = sqrtf(rmse / inliers);
    std::cout << "Rmse RANSAC " << rmse << std::endl;
    return {pose, object_aligned, inliers, rmse};
}

void Plugin::localAlignment()
{
    // ICP (Iterative Closest Point) parameters
    const size_t iter = 50;
    const float threshsq = 0.01 * 0.01;

    auto [pose, object_aligned, local_inliers, local_rmse] = ICP(glob_object_align, iter, threshsq);

    local_pose = pose;
    local_object_align = object_aligned;

    //Set visualize to true to display pointclouds
    bool visualize = false;

    if (visualize) {
        visualizePointClouds(scene_filtered, object_aligned, "After local alignment");
    }
    std::cout << "Finished local alignment!" << std::endl;
    std::cout << "Local transform is:\n" << local_pose << std::endl;
    std::cout << "Rmse Local " << local_rmse << std::endl;
    std::string filename = "Local_RMSE.txt";
    std::ofstream ofs(filename, std::ios::app | std::ofstream::binary);
    ofs << local_rmse;
}

std::tuple<Eigen::Matrix4f, pcl::PointCloud<pcl::PointNormal>::Ptr, size_t, float> Plugin::ICP(pcl::PointCloud<pcl::PointNormal>::Ptr & object, const size_t iter, const float threshsq)
{
    // Create a k-d tree for scene
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(scene_filtered);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned(new pcl::PointCloud<pcl::PointNormal>(*object));

    cout << "Running ICP for " << iter << " iterations." << endl;
    for(size_t i = 0; i < iter; ++i) {

        // Find closest points
        std::vector<std::vector<int> > idx;
        std::vector<std::vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

        // Threshold and create indices for object/scene and compute RMSE
        std::vector<int> idxobj;
        std::vector<int> idxscn;
        for(size_t j = 0; j < idx.size(); ++j) {
            if(distsq[j][0] <= threshsq) {
                idxobj.push_back(j);
                idxscn.push_back(idx[j][0]);
            }
        }

        // Estimate transformation
        Eigen::Matrix4f T;
        pcl::registration::TransformationEstimationSVD<pcl::PointNormal,pcl::PointNormal> est;
        est.estimateRigidTransformation(*object_aligned, idxobj, *scene_filtered, idxscn, T);

        // Apply pose
        pcl::transformPointCloud(*object_aligned, *object_aligned, T);

        // Update result
        pose = T * pose;
    }

    // Compute inliers and RMSE
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;
    tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
    size_t inliers = 0;
    float rmse = 0;
    for(size_t i = 0; i < distsq.size(); ++i)
        if(distsq[i][0] <= threshsq){
            ++inliers;
            rmse += distsq[i][0];
        }
    rmse = sqrtf(rmse / inliers);
    std::cout << "Rmse ICP " << rmse << std::endl;

    return {pose, object_aligned, inliers, rmse};
}

void Plugin::cropScene(pcl::PCLPointCloud2::Ptr inputpcl, pcl::PCLPointCloud2::Ptr & outputpcl, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
    cropFilter.setInputCloud (inputpcl);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.filter(*outputpcl);
}

void Plugin::voxelGrid(pcl::PCLPointCloud2::Ptr inputpcl, pcl::PointCloud<pcl::PointNormal>::Ptr & outputpcl, float leafSize){
    pcl::PCLPointCloud2::Ptr temp (new pcl::PCLPointCloud2 ());

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxobj;
    voxobj.setInputCloud (inputpcl);
    voxobj.setLeafSize (leafSize, leafSize, leafSize);
    voxobj.filter (*temp);

    pcl::fromPCLPointCloud2( *temp, *outputpcl);
}

void Plugin::computeSurfaceNormals(pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, int K)
{
    pcl::NormalEstimation<pcl::PointNormal,pcl::PointNormal> ne;
    ne.setKSearch(K);
    ne.setInputCloud(cloud);
    ne.compute(*cloud);
}

void Plugin::computeShapeFeatures(pcl::PointCloud<pcl::PointNormal>::Ptr & cloud, pcl::PointCloud<pcl::Histogram<153>>::Ptr features, float radius)
{
    pcl::SpinImageEstimation<pcl::PointNormal,pcl::PointNormal,pcl::Histogram<153>> spin;
    spin.setRadiusSearch(radius);
    spin.setInputCloud(cloud);
    spin.setInputNormals(cloud);
    spin.compute(*features);
}

float Plugin::dist_sq(const pcl::Histogram<153>& query, const pcl::Histogram<153>& target) {
    float result = 0.0;
    for(int i = 0; i < pcl::Histogram<153>::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}

void Plugin::nearest_feature(const pcl::Histogram<153>& query, const pcl::PointCloud<pcl::Histogram<153>>& target, int &idx, float &distsq) {

    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

void Plugin::visualizePointClouds(pcl::PointCloud<pcl::PointNormal>::Ptr scene, pcl::PointCloud<pcl::PointNormal>::Ptr object, std::string title) {
    pcl::visualization::PCLVisualizer v(title);
    v.setBackgroundColor(1,1,1);
    v.addPointCloud<pcl::PointNormal>(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene, 0, 0, 255),"scene");
    v.addPointCloud<pcl::PointNormal>(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object, 255, 0, 0), "object");
    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene");
    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object");
    v.spin();
}
