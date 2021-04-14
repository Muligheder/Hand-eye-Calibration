/****************************************************************************
** Meta object code from reading C++ file 'Plugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../Plugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Plugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Plugin_t {
    QByteArrayData data[111];
    char stringdata0[1537];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Plugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Plugin_t qt_meta_stringdata_Plugin = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Plugin"
QT_MOC_LITERAL(1, 7, 10), // "clickEvent"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 20), // "stateChangedListener"
QT_MOC_LITERAL(4, 40, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(5, 62, 5), // "state"
QT_MOC_LITERAL(6, 68, 13), // "RunRobotMimic"
QT_MOC_LITERAL(7, 82, 15), // "RunRobotControl"
QT_MOC_LITERAL(8, 98, 12), // "RunHomeRobot"
QT_MOC_LITERAL(9, 111, 6), // "printQ"
QT_MOC_LITERAL(10, 118, 8), // "printTCP"
QT_MOC_LITERAL(11, 127, 15), // "MoveInToolSpace"
QT_MOC_LITERAL(12, 143, 19), // "TransformationRobot"
QT_MOC_LITERAL(13, 163, 15), // "startRobotMimic"
QT_MOC_LITERAL(14, 179, 17), // "startRobotControl"
QT_MOC_LITERAL(15, 197, 14), // "startHomeRobot"
QT_MOC_LITERAL(16, 212, 6), // "invKin"
QT_MOC_LITERAL(17, 219, 19), // "std::vector<double>"
QT_MOC_LITERAL(18, 239, 12), // "connectRobot"
QT_MOC_LITERAL(19, 252, 9), // "stopRobot"
QT_MOC_LITERAL(20, 262, 8), // "stopSync"
QT_MOC_LITERAL(21, 271, 15), // "teachModeToggle"
QT_MOC_LITERAL(22, 287, 15), // "getConfDistance"
QT_MOC_LITERAL(23, 303, 10), // "printArray"
QT_MOC_LITERAL(24, 314, 7), // "addMove"
QT_MOC_LITERAL(25, 322, 20), // "write_vector_to_file"
QT_MOC_LITERAL(26, 343, 8), // "myVector"
QT_MOC_LITERAL(27, 352, 11), // "std::string"
QT_MOC_LITERAL(28, 364, 8), // "filename"
QT_MOC_LITERAL(29, 373, 16), // "printDeviceNames"
QT_MOC_LITERAL(30, 390, 20), // "rw::models::WorkCell"
QT_MOC_LITERAL(31, 411, 8), // "workcell"
QT_MOC_LITERAL(32, 420, 7), // "moveToJ"
QT_MOC_LITERAL(33, 428, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(34, 449, 34), // "std::vector<std::vector<doubl..."
QT_MOC_LITERAL(35, 484, 12), // "Take_picture"
QT_MOC_LITERAL(36, 497, 14), // "Analyze_images"
QT_MOC_LITERAL(37, 512, 3), // "PCL"
QT_MOC_LITERAL(38, 516, 15), // "globalAlignment"
QT_MOC_LITERAL(39, 532, 14), // "localAlignment"
QT_MOC_LITERAL(40, 547, 9), // "cropScene"
QT_MOC_LITERAL(41, 557, 24), // "pcl::PCLPointCloud2::Ptr"
QT_MOC_LITERAL(42, 582, 8), // "inputpcl"
QT_MOC_LITERAL(43, 591, 25), // "pcl::PCLPointCloud2::Ptr&"
QT_MOC_LITERAL(44, 617, 9), // "outputpcl"
QT_MOC_LITERAL(45, 627, 15), // "Eigen::Vector4f"
QT_MOC_LITERAL(46, 643, 8), // "minPoint"
QT_MOC_LITERAL(47, 652, 8), // "maxPoint"
QT_MOC_LITERAL(48, 661, 9), // "voxelGrid"
QT_MOC_LITERAL(49, 671, 39), // "pcl::PointCloud<pcl::PointNor..."
QT_MOC_LITERAL(50, 711, 8), // "leafSize"
QT_MOC_LITERAL(51, 720, 21), // "computeSurfaceNormals"
QT_MOC_LITERAL(52, 742, 5), // "cloud"
QT_MOC_LITERAL(53, 748, 1), // "K"
QT_MOC_LITERAL(54, 750, 20), // "computeShapeFeatures"
QT_MOC_LITERAL(55, 771, 42), // "pcl::PointCloud<pcl::Histogra..."
QT_MOC_LITERAL(56, 814, 8), // "features"
QT_MOC_LITERAL(57, 823, 6), // "radius"
QT_MOC_LITERAL(58, 830, 6), // "RANSAC"
QT_MOC_LITERAL(59, 837, 79), // "std::tuple<Eigen::Matrix4f,pc..."
QT_MOC_LITERAL(60, 917, 6), // "object"
QT_MOC_LITERAL(61, 924, 20), // "pcl::Correspondences"
QT_MOC_LITERAL(62, 945, 4), // "corr"
QT_MOC_LITERAL(63, 950, 6), // "size_t"
QT_MOC_LITERAL(64, 957, 4), // "iter"
QT_MOC_LITERAL(65, 962, 8), // "threshsq"
QT_MOC_LITERAL(66, 971, 3), // "ICP"
QT_MOC_LITERAL(67, 975, 15), // "nearest_feature"
QT_MOC_LITERAL(68, 991, 19), // "pcl::Histogram<153>"
QT_MOC_LITERAL(69, 1011, 5), // "query"
QT_MOC_LITERAL(70, 1017, 37), // "pcl::PointCloud<pcl::Histogra..."
QT_MOC_LITERAL(71, 1055, 6), // "target"
QT_MOC_LITERAL(72, 1062, 4), // "int&"
QT_MOC_LITERAL(73, 1067, 3), // "idx"
QT_MOC_LITERAL(74, 1071, 6), // "float&"
QT_MOC_LITERAL(75, 1078, 6), // "distsq"
QT_MOC_LITERAL(76, 1085, 7), // "dist_sq"
QT_MOC_LITERAL(77, 1093, 20), // "visualizePointClouds"
QT_MOC_LITERAL(78, 1114, 38), // "pcl::PointCloud<pcl::PointNor..."
QT_MOC_LITERAL(79, 1153, 5), // "scene"
QT_MOC_LITERAL(80, 1159, 5), // "title"
QT_MOC_LITERAL(81, 1165, 27), // "eulerAnglesToRotationMatrix"
QT_MOC_LITERAL(82, 1193, 7), // "cv::Mat"
QT_MOC_LITERAL(83, 1201, 10), // "cv::Vec3d&"
QT_MOC_LITERAL(84, 1212, 5), // "theta"
QT_MOC_LITERAL(85, 1218, 7), // "rad2deg"
QT_MOC_LITERAL(86, 1226, 6), // "radian"
QT_MOC_LITERAL(87, 1233, 7), // "deg2rad"
QT_MOC_LITERAL(88, 1241, 6), // "degree"
QT_MOC_LITERAL(89, 1248, 16), // "isRotationMatrix"
QT_MOC_LITERAL(90, 1265, 8), // "cv::Mat&"
QT_MOC_LITERAL(91, 1274, 1), // "R"
QT_MOC_LITERAL(92, 1276, 27), // "rotationMatrixToEulerAngles"
QT_MOC_LITERAL(93, 1304, 9), // "cv::Vec3d"
QT_MOC_LITERAL(94, 1314, 13), // "ReverseVector"
QT_MOC_LITERAL(95, 1328, 1), // "v"
QT_MOC_LITERAL(96, 1330, 8), // "Pose_inv"
QT_MOC_LITERAL(97, 1339, 13), // "import3DPoint"
QT_MOC_LITERAL(98, 1353, 20), // "std::vector<double>&"
QT_MOC_LITERAL(99, 1374, 5), // "point"
QT_MOC_LITERAL(100, 1380, 12), // "get_texcolor"
QT_MOC_LITERAL(101, 1393, 35), // "std::tuple<uint8_t,uint8_t,ui..."
QT_MOC_LITERAL(102, 1429, 16), // "rs2::video_frame"
QT_MOC_LITERAL(103, 1446, 7), // "texture"
QT_MOC_LITERAL(104, 1454, 23), // "rs2::texture_coordinate"
QT_MOC_LITERAL(105, 1478, 9), // "texcoords"
QT_MOC_LITERAL(106, 1488, 13), // "points_to_pcl"
QT_MOC_LITERAL(107, 1502, 9), // "ptr_cloud"
QT_MOC_LITERAL(108, 1512, 11), // "rs2::points"
QT_MOC_LITERAL(109, 1524, 6), // "points"
QT_MOC_LITERAL(110, 1531, 5) // "color"

    },
    "Plugin\0clickEvent\0\0stateChangedListener\0"
    "rw::kinematics::State\0state\0RunRobotMimic\0"
    "RunRobotControl\0RunHomeRobot\0printQ\0"
    "printTCP\0MoveInToolSpace\0TransformationRobot\0"
    "startRobotMimic\0startRobotControl\0"
    "startHomeRobot\0invKin\0std::vector<double>\0"
    "connectRobot\0stopRobot\0stopSync\0"
    "teachModeToggle\0getConfDistance\0"
    "printArray\0addMove\0write_vector_to_file\0"
    "myVector\0std::string\0filename\0"
    "printDeviceNames\0rw::models::WorkCell\0"
    "workcell\0moveToJ\0createPathRRTConnect\0"
    "std::vector<std::vector<double> >&\0"
    "Take_picture\0Analyze_images\0PCL\0"
    "globalAlignment\0localAlignment\0cropScene\0"
    "pcl::PCLPointCloud2::Ptr\0inputpcl\0"
    "pcl::PCLPointCloud2::Ptr&\0outputpcl\0"
    "Eigen::Vector4f\0minPoint\0maxPoint\0"
    "voxelGrid\0pcl::PointCloud<pcl::PointNormal>::Ptr&\0"
    "leafSize\0computeSurfaceNormals\0cloud\0"
    "K\0computeShapeFeatures\0"
    "pcl::PointCloud<pcl::Histogram<153> >::Ptr\0"
    "features\0radius\0RANSAC\0"
    "std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size"
    "_t,float>\0"
    "object\0pcl::Correspondences\0corr\0"
    "size_t\0iter\0threshsq\0ICP\0nearest_feature\0"
    "pcl::Histogram<153>\0query\0"
    "pcl::PointCloud<pcl::Histogram<153> >\0"
    "target\0int&\0idx\0float&\0distsq\0dist_sq\0"
    "visualizePointClouds\0"
    "pcl::PointCloud<pcl::PointNormal>::Ptr\0"
    "scene\0title\0eulerAnglesToRotationMatrix\0"
    "cv::Mat\0cv::Vec3d&\0theta\0rad2deg\0"
    "radian\0deg2rad\0degree\0isRotationMatrix\0"
    "cv::Mat&\0R\0rotationMatrixToEulerAngles\0"
    "cv::Vec3d\0ReverseVector\0v\0Pose_inv\0"
    "import3DPoint\0std::vector<double>&\0"
    "point\0get_texcolor\0"
    "std::tuple<uint8_t,uint8_t,uint8_t>\0"
    "rs2::video_frame\0texture\0"
    "rs2::texture_coordinate\0texcoords\0"
    "points_to_pcl\0ptr_cloud\0rs2::points\0"
    "points\0color"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Plugin[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      51,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  269,    2, 0x08 /* Private */,
       3,    1,  270,    2, 0x08 /* Private */,
       6,    0,  273,    2, 0x08 /* Private */,
       7,    0,  274,    2, 0x08 /* Private */,
       8,    0,  275,    2, 0x08 /* Private */,
       9,    0,  276,    2, 0x08 /* Private */,
      10,    0,  277,    2, 0x08 /* Private */,
      11,    0,  278,    2, 0x08 /* Private */,
      12,    0,  279,    2, 0x08 /* Private */,
      13,    0,  280,    2, 0x08 /* Private */,
      14,    0,  281,    2, 0x08 /* Private */,
      15,    0,  282,    2, 0x08 /* Private */,
      16,    2,  283,    2, 0x08 /* Private */,
      18,    0,  288,    2, 0x08 /* Private */,
      19,    0,  289,    2, 0x08 /* Private */,
      20,    0,  290,    2, 0x08 /* Private */,
      21,    0,  291,    2, 0x08 /* Private */,
      22,    2,  292,    2, 0x08 /* Private */,
      23,    1,  297,    2, 0x08 /* Private */,
      24,    4,  300,    2, 0x08 /* Private */,
      25,    2,  309,    2, 0x08 /* Private */,
      29,    1,  314,    2, 0x08 /* Private */,
      32,    3,  317,    2, 0x08 /* Private */,
      33,    8,  324,    2, 0x08 /* Private */,
      35,    0,  341,    2, 0x08 /* Private */,
      36,    0,  342,    2, 0x08 /* Private */,
      37,    0,  343,    2, 0x08 /* Private */,
      38,    0,  344,    2, 0x08 /* Private */,
      39,    0,  345,    2, 0x08 /* Private */,
      40,    4,  346,    2, 0x08 /* Private */,
      48,    3,  355,    2, 0x08 /* Private */,
      48,    2,  362,    2, 0x28 /* Private | MethodCloned */,
      51,    2,  367,    2, 0x08 /* Private */,
      51,    1,  372,    2, 0x28 /* Private | MethodCloned */,
      54,    3,  375,    2, 0x08 /* Private */,
      54,    2,  382,    2, 0x28 /* Private | MethodCloned */,
      58,    4,  387,    2, 0x08 /* Private */,
      66,    3,  396,    2, 0x08 /* Private */,
      67,    4,  403,    2, 0x08 /* Private */,
      76,    2,  412,    2, 0x08 /* Private */,
      77,    3,  417,    2, 0x08 /* Private */,
      81,    1,  424,    2, 0x08 /* Private */,
      85,    1,  427,    2, 0x08 /* Private */,
      87,    1,  430,    2, 0x08 /* Private */,
      89,    1,  433,    2, 0x08 /* Private */,
      92,    1,  436,    2, 0x08 /* Private */,
      94,    1,  439,    2, 0x08 /* Private */,
      96,    1,  442,    2, 0x08 /* Private */,
      97,    1,  445,    2, 0x08 /* Private */,
     100,    2,  448,    2, 0x08 /* Private */,
     106,    2,  453,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 17, 0x80000000 | 17, 0x80000000 | 17,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Double, 0x80000000 | 17, 0x80000000 | 17,    2,    2,
    QMetaType::Void, 0x80000000 | 17,    2,
    0x80000000 | 17, 0x80000000 | 17, QMetaType::Double, QMetaType::Double, QMetaType::Double,    2,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 17, 0x80000000 | 27,   26,   28,
    QMetaType::Void, 0x80000000 | 30,   31,
    QMetaType::Void, 0x80000000 | 17, QMetaType::Double, QMetaType::Double,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 17, 0x80000000 | 17, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, 0x80000000 | 34, 0x80000000 | 4,    2,    2,    2,    2,    2,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 41, 0x80000000 | 43, 0x80000000 | 45, 0x80000000 | 45,   42,   44,   46,   47,
    QMetaType::Void, 0x80000000 | 41, 0x80000000 | 49, QMetaType::Float,   42,   44,   50,
    QMetaType::Void, 0x80000000 | 41, 0x80000000 | 49,   42,   44,
    QMetaType::Void, 0x80000000 | 49, QMetaType::Int,   52,   53,
    QMetaType::Void, 0x80000000 | 49,   52,
    QMetaType::Void, 0x80000000 | 49, 0x80000000 | 55, QMetaType::Float,   52,   56,   57,
    QMetaType::Void, 0x80000000 | 49, 0x80000000 | 55,   52,   56,
    0x80000000 | 59, 0x80000000 | 49, 0x80000000 | 61, 0x80000000 | 63, QMetaType::Float,   60,   62,   64,   65,
    0x80000000 | 59, 0x80000000 | 49, 0x80000000 | 63, QMetaType::Float,   60,   64,   65,
    QMetaType::Void, 0x80000000 | 68, 0x80000000 | 70, 0x80000000 | 72, 0x80000000 | 74,   69,   71,   73,   75,
    QMetaType::Float, 0x80000000 | 68, 0x80000000 | 68,   69,   71,
    QMetaType::Void, 0x80000000 | 78, 0x80000000 | 78, 0x80000000 | 27,   79,   60,   80,
    0x80000000 | 82, 0x80000000 | 83,   84,
    QMetaType::Double, QMetaType::Double,   86,
    QMetaType::Double, QMetaType::Double,   88,
    QMetaType::Bool, 0x80000000 | 90,   91,
    0x80000000 | 93, 0x80000000 | 90,   91,
    0x80000000 | 82, 0x80000000 | 90,   95,
    0x80000000 | 17, 0x80000000 | 17,   95,
    QMetaType::Void, 0x80000000 | 98,   99,
    0x80000000 | 101, 0x80000000 | 102, 0x80000000 | 104,  103,  105,
    0x80000000 | 107, 0x80000000 | 108, 0x80000000 | 102,  109,  110,

       0        // eod
};

void Plugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Plugin *_t = static_cast<Plugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->clickEvent(); break;
        case 1: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        case 2: _t->RunRobotMimic(); break;
        case 3: _t->RunRobotControl(); break;
        case 4: _t->RunHomeRobot(); break;
        case 5: _t->printQ(); break;
        case 6: _t->printTCP(); break;
        case 7: _t->MoveInToolSpace(); break;
        case 8: _t->TransformationRobot(); break;
        case 9: _t->startRobotMimic(); break;
        case 10: _t->startRobotControl(); break;
        case 11: _t->startHomeRobot(); break;
        case 12: { std::vector<double> _r = _t->invKin((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 13: _t->connectRobot(); break;
        case 14: _t->stopRobot(); break;
        case 15: _t->stopSync(); break;
        case 16: _t->teachModeToggle(); break;
        case 17: { double _r = _t->getConfDistance((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 18: _t->printArray((*reinterpret_cast< std::vector<double>(*)>(_a[1]))); break;
        case 19: { std::vector<double> _r = _t->addMove((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 20: _t->write_vector_to_file((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 21: _t->printDeviceNames((*reinterpret_cast< const rw::models::WorkCell(*)>(_a[1]))); break;
        case 22: _t->moveToJ((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 23: _t->createPathRRTConnect((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< std::vector<std::vector<double> >(*)>(_a[7])),(*reinterpret_cast< rw::kinematics::State(*)>(_a[8]))); break;
        case 24: _t->Take_picture(); break;
        case 25: _t->Analyze_images(); break;
        case 26: _t->PCL(); break;
        case 27: _t->globalAlignment(); break;
        case 28: _t->localAlignment(); break;
        case 29: _t->cropScene((*reinterpret_cast< pcl::PCLPointCloud2::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PCLPointCloud2::Ptr(*)>(_a[2])),(*reinterpret_cast< Eigen::Vector4f(*)>(_a[3])),(*reinterpret_cast< Eigen::Vector4f(*)>(_a[4]))); break;
        case 30: _t->voxelGrid((*reinterpret_cast< pcl::PCLPointCloud2::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 31: _t->voxelGrid((*reinterpret_cast< pcl::PCLPointCloud2::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[2]))); break;
        case 32: _t->computeSurfaceNormals((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 33: _t->computeSurfaceNormals((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1]))); break;
        case 34: _t->computeShapeFeatures((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::Histogram<153> >::Ptr(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 35: _t->computeShapeFeatures((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::Histogram<153> >::Ptr(*)>(_a[2]))); break;
        case 36: { std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float> _r = _t->RANSAC((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::Correspondences(*)>(_a[2])),(*reinterpret_cast< const size_t(*)>(_a[3])),(*reinterpret_cast< const float(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float>*>(_a[0]) = std::move(_r); }  break;
        case 37: { std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float> _r = _t->ICP((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< const size_t(*)>(_a[2])),(*reinterpret_cast< const float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float>*>(_a[0]) = std::move(_r); }  break;
        case 38: _t->nearest_feature((*reinterpret_cast< const pcl::Histogram<153>(*)>(_a[1])),(*reinterpret_cast< const pcl::PointCloud<pcl::Histogram<153> >(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        case 39: { float _r = _t->dist_sq((*reinterpret_cast< const pcl::Histogram<153>(*)>(_a[1])),(*reinterpret_cast< const pcl::Histogram<153>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = std::move(_r); }  break;
        case 40: _t->visualizePointClouds((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[2])),(*reinterpret_cast< std::string(*)>(_a[3]))); break;
        case 41: { cv::Mat _r = _t->eulerAnglesToRotationMatrix((*reinterpret_cast< cv::Vec3d(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 42: { double _r = _t->rad2deg((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 43: { double _r = _t->deg2rad((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 44: { bool _r = _t->isRotationMatrix((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 45: { cv::Vec3d _r = _t->rotationMatrixToEulerAngles((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Vec3d*>(_a[0]) = std::move(_r); }  break;
        case 46: { cv::Mat _r = _t->ReverseVector((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 47: { std::vector<double> _r = _t->Pose_inv((*reinterpret_cast< std::vector<double>(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 48: _t->import3DPoint((*reinterpret_cast< std::vector<double>(*)>(_a[1]))); break;
        case 49: { std::tuple<uint8_t,uint8_t,uint8_t> _r = _t->get_texcolor((*reinterpret_cast< rs2::video_frame(*)>(_a[1])),(*reinterpret_cast< rs2::texture_coordinate(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::tuple<uint8_t,uint8_t,uint8_t>*>(_a[0]) = std::move(_r); }  break;
        case 50: { ptr_cloud _r = _t->points_to_pcl((*reinterpret_cast< const rs2::points(*)>(_a[1])),(*reinterpret_cast< const rs2::video_frame(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< ptr_cloud*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject Plugin::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_Plugin.data,
      qt_meta_data_Plugin,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Plugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Plugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Plugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int Plugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 51)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 51;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 51)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 51;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION const uint qt_section_alignment_dummy = 42;

#ifdef QT_NO_DEBUG

QT_PLUGIN_METADATA_SECTION
static const unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', ' ',
    'q',  'b',  'j',  's',  0x01, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
    0xec, 0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x03, 0x00, 'I',  'I',  'D',  0x00, 0x00, 0x00,
    '*',  0x00, 'd',  'k',  '.',  's',  'd',  'u', 
    '.',  'm',  'i',  'p',  '.',  'R',  'o',  'b', 
    'w',  'o',  'r',  'k',  '.',  'R',  'o',  'b', 
    'W',  'o',  'r',  'k',  'S',  't',  'u',  'd', 
    'i',  'o',  'P',  'l',  'u',  'g',  'i',  'n', 
    '/',  '0',  '.',  '1',  0x9b, 0x0a, 0x00, 0x00,
    0x09, 0x00, 'c',  'l',  'a',  's',  's',  'N', 
    'a',  'm',  'e',  0x00, 0x06, 0x00, 'P',  'l', 
    'u',  'g',  'i',  'n',  0xba, ' ',  0xa1, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x05, 0x00, 'd',  'e',  'b',  'u',  'g',  0x00,
    0x15, 0x11, 0x00, 0x00, 0x08, 0x00, 'M',  'e', 
    't',  'a',  'D',  'a',  't',  'a',  0x00, 0x00,
    'd',  0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    'X',  0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x04, 0x00, 'n',  'a',  'm',  'e',  0x00, 0x00,
    0x06, 0x00, 'P',  'l',  'u',  'g',  'i',  'n', 
    0x1b, 0x06, 0x00, 0x00, 0x07, 0x00, 'v',  'e', 
    'r',  's',  'i',  'o',  'n',  0x00, 0x00, 0x00,
    0x05, 0x00, '1',  '.',  '0',  '.',  '0',  0x00,
    0x94, 0x09, 0x00, 0x00, 0x0c, 0x00, 'd',  'e', 
    'p',  'e',  'n',  'd',  'e',  'n',  'c',  'i', 
    'e',  's',  0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    '8',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    ' ',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    'x',  0x00, 0x00, 0x00, 'D',  0x00, 0x00, 0x00,
    'l',  0x00, 0x00, 0x00, '\\', 0x00, 0x00, 0x00
};

#else // QT_NO_DEBUG

QT_PLUGIN_METADATA_SECTION
static const unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', ' ',
    'q',  'b',  'j',  's',  0x01, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
    0xec, 0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x03, 0x00, 'I',  'I',  'D',  0x00, 0x00, 0x00,
    '*',  0x00, 'd',  'k',  '.',  's',  'd',  'u', 
    '.',  'm',  'i',  'p',  '.',  'R',  'o',  'b', 
    'w',  'o',  'r',  'k',  '.',  'R',  'o',  'b', 
    'W',  'o',  'r',  'k',  'S',  't',  'u',  'd', 
    'i',  'o',  'P',  'l',  'u',  'g',  'i',  'n', 
    '/',  '0',  '.',  '1',  0x95, 0x0a, 0x00, 0x00,
    0x08, 0x00, 'M',  'e',  't',  'a',  'D',  'a', 
    't',  'a',  0x00, 0x00, 'd',  0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 'X',  0x00, 0x00, 0x00,
    0x1b, 0x03, 0x00, 0x00, 0x04, 0x00, 'n',  'a', 
    'm',  'e',  0x00, 0x00, 0x06, 0x00, 'P',  'l', 
    'u',  'g',  'i',  'n',  0x1b, 0x06, 0x00, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x05, 0x00, '1',  '.', 
    '0',  '.',  '0',  0x00, 0x94, 0x09, 0x00, 0x00,
    0x0c, 0x00, 'd',  'e',  'p',  'e',  'n',  'd', 
    'e',  'n',  'c',  'i',  'e',  's',  0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, '8',  0x00, 0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, ' ',  0x00, 0x00, 0x00,
    0x1b, 0x19, 0x00, 0x00, 0x09, 0x00, 'c',  'l', 
    'a',  's',  's',  'N',  'a',  'm',  'e',  0x00,
    0x06, 0x00, 'P',  'l',  'u',  'g',  'i',  'n', 
    '1',  0x00, 0x00, 0x00, 0x05, 0x00, 'd',  'e', 
    'b',  'u',  'g',  0x00, 0xba, ' ',  0xa1, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    'D',  0x00, 0x00, 0x00, 0xb8, 0x00, 0x00, 0x00,
    0xd0, 0x00, 0x00, 0x00, 0xdc, 0x00, 0x00, 0x00
};
#endif // QT_NO_DEBUG

QT_MOC_EXPORT_PLUGIN(Plugin, Plugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
