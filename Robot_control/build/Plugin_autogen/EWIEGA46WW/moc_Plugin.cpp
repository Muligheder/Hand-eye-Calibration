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
    QByteArrayData data[112];
    char stringdata0[1552];
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
QT_MOC_LITERAL(51, 720, 14), // "outlierRemoval"
QT_MOC_LITERAL(52, 735, 38), // "pcl::PointCloud<pcl::PointNor..."
QT_MOC_LITERAL(53, 774, 21), // "computeSurfaceNormals"
QT_MOC_LITERAL(54, 796, 5), // "cloud"
QT_MOC_LITERAL(55, 802, 1), // "K"
QT_MOC_LITERAL(56, 804, 20), // "computeShapeFeatures"
QT_MOC_LITERAL(57, 825, 42), // "pcl::PointCloud<pcl::Histogra..."
QT_MOC_LITERAL(58, 868, 8), // "features"
QT_MOC_LITERAL(59, 877, 6), // "radius"
QT_MOC_LITERAL(60, 884, 6), // "RANSAC"
QT_MOC_LITERAL(61, 891, 79), // "std::tuple<Eigen::Matrix4f,pc..."
QT_MOC_LITERAL(62, 971, 6), // "object"
QT_MOC_LITERAL(63, 978, 20), // "pcl::Correspondences"
QT_MOC_LITERAL(64, 999, 4), // "corr"
QT_MOC_LITERAL(65, 1004, 6), // "size_t"
QT_MOC_LITERAL(66, 1011, 4), // "iter"
QT_MOC_LITERAL(67, 1016, 8), // "threshsq"
QT_MOC_LITERAL(68, 1025, 3), // "ICP"
QT_MOC_LITERAL(69, 1029, 15), // "nearest_feature"
QT_MOC_LITERAL(70, 1045, 19), // "pcl::Histogram<153>"
QT_MOC_LITERAL(71, 1065, 5), // "query"
QT_MOC_LITERAL(72, 1071, 37), // "pcl::PointCloud<pcl::Histogra..."
QT_MOC_LITERAL(73, 1109, 6), // "target"
QT_MOC_LITERAL(74, 1116, 4), // "int&"
QT_MOC_LITERAL(75, 1121, 3), // "idx"
QT_MOC_LITERAL(76, 1125, 6), // "float&"
QT_MOC_LITERAL(77, 1132, 6), // "distsq"
QT_MOC_LITERAL(78, 1139, 7), // "dist_sq"
QT_MOC_LITERAL(79, 1147, 20), // "visualizePointClouds"
QT_MOC_LITERAL(80, 1168, 5), // "scene"
QT_MOC_LITERAL(81, 1174, 5), // "title"
QT_MOC_LITERAL(82, 1180, 27), // "eulerAnglesToRotationMatrix"
QT_MOC_LITERAL(83, 1208, 7), // "cv::Mat"
QT_MOC_LITERAL(84, 1216, 10), // "cv::Vec3d&"
QT_MOC_LITERAL(85, 1227, 5), // "theta"
QT_MOC_LITERAL(86, 1233, 7), // "rad2deg"
QT_MOC_LITERAL(87, 1241, 6), // "radian"
QT_MOC_LITERAL(88, 1248, 7), // "deg2rad"
QT_MOC_LITERAL(89, 1256, 6), // "degree"
QT_MOC_LITERAL(90, 1263, 16), // "isRotationMatrix"
QT_MOC_LITERAL(91, 1280, 8), // "cv::Mat&"
QT_MOC_LITERAL(92, 1289, 1), // "R"
QT_MOC_LITERAL(93, 1291, 27), // "rotationMatrixToEulerAngles"
QT_MOC_LITERAL(94, 1319, 9), // "cv::Vec3d"
QT_MOC_LITERAL(95, 1329, 13), // "ReverseVector"
QT_MOC_LITERAL(96, 1343, 1), // "v"
QT_MOC_LITERAL(97, 1345, 8), // "Pose_inv"
QT_MOC_LITERAL(98, 1354, 13), // "import3DPoint"
QT_MOC_LITERAL(99, 1368, 20), // "std::vector<double>&"
QT_MOC_LITERAL(100, 1389, 5), // "point"
QT_MOC_LITERAL(101, 1395, 12), // "get_texcolor"
QT_MOC_LITERAL(102, 1408, 35), // "std::tuple<uint8_t,uint8_t,ui..."
QT_MOC_LITERAL(103, 1444, 16), // "rs2::video_frame"
QT_MOC_LITERAL(104, 1461, 7), // "texture"
QT_MOC_LITERAL(105, 1469, 23), // "rs2::texture_coordinate"
QT_MOC_LITERAL(106, 1493, 9), // "texcoords"
QT_MOC_LITERAL(107, 1503, 13), // "points_to_pcl"
QT_MOC_LITERAL(108, 1517, 9), // "ptr_cloud"
QT_MOC_LITERAL(109, 1527, 11), // "rs2::points"
QT_MOC_LITERAL(110, 1539, 6), // "points"
QT_MOC_LITERAL(111, 1546, 5) // "color"

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
    "leafSize\0outlierRemoval\0"
    "pcl::PointCloud<pcl::PointNormal>::Ptr\0"
    "computeSurfaceNormals\0cloud\0K\0"
    "computeShapeFeatures\0"
    "pcl::PointCloud<pcl::Histogram<153> >::Ptr\0"
    "features\0radius\0RANSAC\0"
    "std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size"
    "_t,float>\0"
    "object\0pcl::Correspondences\0corr\0"
    "size_t\0iter\0threshsq\0ICP\0nearest_feature\0"
    "pcl::Histogram<153>\0query\0"
    "pcl::PointCloud<pcl::Histogram<153> >\0"
    "target\0int&\0idx\0float&\0distsq\0dist_sq\0"
    "visualizePointClouds\0scene\0title\0"
    "eulerAnglesToRotationMatrix\0cv::Mat\0"
    "cv::Vec3d&\0theta\0rad2deg\0radian\0deg2rad\0"
    "degree\0isRotationMatrix\0cv::Mat&\0R\0"
    "rotationMatrixToEulerAngles\0cv::Vec3d\0"
    "ReverseVector\0v\0Pose_inv\0import3DPoint\0"
    "std::vector<double>&\0point\0get_texcolor\0"
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
      52,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  274,    2, 0x08 /* Private */,
       3,    1,  275,    2, 0x08 /* Private */,
       6,    0,  278,    2, 0x08 /* Private */,
       7,    0,  279,    2, 0x08 /* Private */,
       8,    0,  280,    2, 0x08 /* Private */,
       9,    0,  281,    2, 0x08 /* Private */,
      10,    0,  282,    2, 0x08 /* Private */,
      11,    0,  283,    2, 0x08 /* Private */,
      12,    0,  284,    2, 0x08 /* Private */,
      13,    0,  285,    2, 0x08 /* Private */,
      14,    0,  286,    2, 0x08 /* Private */,
      15,    0,  287,    2, 0x08 /* Private */,
      16,    2,  288,    2, 0x08 /* Private */,
      18,    0,  293,    2, 0x08 /* Private */,
      19,    0,  294,    2, 0x08 /* Private */,
      20,    0,  295,    2, 0x08 /* Private */,
      21,    0,  296,    2, 0x08 /* Private */,
      22,    2,  297,    2, 0x08 /* Private */,
      23,    1,  302,    2, 0x08 /* Private */,
      24,    4,  305,    2, 0x08 /* Private */,
      25,    2,  314,    2, 0x08 /* Private */,
      29,    1,  319,    2, 0x08 /* Private */,
      32,    3,  322,    2, 0x08 /* Private */,
      33,    8,  329,    2, 0x08 /* Private */,
      35,    0,  346,    2, 0x08 /* Private */,
      36,    0,  347,    2, 0x08 /* Private */,
      37,    0,  348,    2, 0x08 /* Private */,
      38,    0,  349,    2, 0x08 /* Private */,
      39,    0,  350,    2, 0x08 /* Private */,
      40,    4,  351,    2, 0x08 /* Private */,
      48,    3,  360,    2, 0x08 /* Private */,
      48,    2,  367,    2, 0x28 /* Private | MethodCloned */,
      51,    2,  372,    2, 0x08 /* Private */,
      53,    2,  377,    2, 0x08 /* Private */,
      53,    1,  382,    2, 0x28 /* Private | MethodCloned */,
      56,    3,  385,    2, 0x08 /* Private */,
      56,    2,  392,    2, 0x28 /* Private | MethodCloned */,
      60,    4,  397,    2, 0x08 /* Private */,
      68,    3,  406,    2, 0x08 /* Private */,
      69,    4,  413,    2, 0x08 /* Private */,
      78,    2,  422,    2, 0x08 /* Private */,
      79,    3,  427,    2, 0x08 /* Private */,
      82,    1,  434,    2, 0x08 /* Private */,
      86,    1,  437,    2, 0x08 /* Private */,
      88,    1,  440,    2, 0x08 /* Private */,
      90,    1,  443,    2, 0x08 /* Private */,
      93,    1,  446,    2, 0x08 /* Private */,
      95,    1,  449,    2, 0x08 /* Private */,
      97,    1,  452,    2, 0x08 /* Private */,
      98,    1,  455,    2, 0x08 /* Private */,
     101,    2,  458,    2, 0x08 /* Private */,
     107,    2,  463,    2, 0x08 /* Private */,

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
    QMetaType::Void, 0x80000000 | 52, 0x80000000 | 49,   42,   44,
    QMetaType::Void, 0x80000000 | 49, QMetaType::Int,   54,   55,
    QMetaType::Void, 0x80000000 | 49,   54,
    QMetaType::Void, 0x80000000 | 49, 0x80000000 | 57, QMetaType::Float,   54,   58,   59,
    QMetaType::Void, 0x80000000 | 49, 0x80000000 | 57,   54,   58,
    0x80000000 | 61, 0x80000000 | 49, 0x80000000 | 63, 0x80000000 | 65, QMetaType::Float,   62,   64,   66,   67,
    0x80000000 | 61, 0x80000000 | 49, 0x80000000 | 65, QMetaType::Float,   62,   66,   67,
    QMetaType::Void, 0x80000000 | 70, 0x80000000 | 72, 0x80000000 | 74, 0x80000000 | 76,   71,   73,   75,   77,
    QMetaType::Float, 0x80000000 | 70, 0x80000000 | 70,   71,   73,
    QMetaType::Void, 0x80000000 | 52, 0x80000000 | 52, 0x80000000 | 27,   80,   62,   81,
    0x80000000 | 83, 0x80000000 | 84,   85,
    QMetaType::Double, QMetaType::Double,   87,
    QMetaType::Double, QMetaType::Double,   89,
    QMetaType::Bool, 0x80000000 | 91,   92,
    0x80000000 | 94, 0x80000000 | 91,   92,
    0x80000000 | 83, 0x80000000 | 91,   96,
    0x80000000 | 17, 0x80000000 | 17,   96,
    QMetaType::Void, 0x80000000 | 99,  100,
    0x80000000 | 102, 0x80000000 | 103, 0x80000000 | 105,  104,  106,
    0x80000000 | 108, 0x80000000 | 109, 0x80000000 | 103,  110,  111,

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
        case 32: _t->outlierRemoval((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[2]))); break;
        case 33: _t->computeSurfaceNormals((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 34: _t->computeSurfaceNormals((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1]))); break;
        case 35: _t->computeShapeFeatures((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::Histogram<153> >::Ptr(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 36: _t->computeShapeFeatures((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::Histogram<153> >::Ptr(*)>(_a[2]))); break;
        case 37: { std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float> _r = _t->RANSAC((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::Correspondences(*)>(_a[2])),(*reinterpret_cast< const size_t(*)>(_a[3])),(*reinterpret_cast< const float(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float>*>(_a[0]) = std::move(_r); }  break;
        case 38: { std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float> _r = _t->ICP((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< const size_t(*)>(_a[2])),(*reinterpret_cast< const float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< std::tuple<Eigen::Matrix4f,pcl::PointCloud<pcl::PointNormal>::Ptr,size_t,float>*>(_a[0]) = std::move(_r); }  break;
        case 39: _t->nearest_feature((*reinterpret_cast< const pcl::Histogram<153>(*)>(_a[1])),(*reinterpret_cast< const pcl::PointCloud<pcl::Histogram<153> >(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        case 40: { float _r = _t->dist_sq((*reinterpret_cast< const pcl::Histogram<153>(*)>(_a[1])),(*reinterpret_cast< const pcl::Histogram<153>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = std::move(_r); }  break;
        case 41: _t->visualizePointClouds((*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointNormal>::Ptr(*)>(_a[2])),(*reinterpret_cast< std::string(*)>(_a[3]))); break;
        case 42: { cv::Mat _r = _t->eulerAnglesToRotationMatrix((*reinterpret_cast< cv::Vec3d(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 43: { double _r = _t->rad2deg((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 44: { double _r = _t->deg2rad((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 45: { bool _r = _t->isRotationMatrix((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 46: { cv::Vec3d _r = _t->rotationMatrixToEulerAngles((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Vec3d*>(_a[0]) = std::move(_r); }  break;
        case 47: { cv::Mat _r = _t->ReverseVector((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 48: { std::vector<double> _r = _t->Pose_inv((*reinterpret_cast< std::vector<double>(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 49: _t->import3DPoint((*reinterpret_cast< std::vector<double>(*)>(_a[1]))); break;
        case 50: { std::tuple<uint8_t,uint8_t,uint8_t> _r = _t->get_texcolor((*reinterpret_cast< rs2::video_frame(*)>(_a[1])),(*reinterpret_cast< rs2::texture_coordinate(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::tuple<uint8_t,uint8_t,uint8_t>*>(_a[0]) = std::move(_r); }  break;
        case 51: { ptr_cloud _r = _t->points_to_pcl((*reinterpret_cast< const rs2::points(*)>(_a[1])),(*reinterpret_cast< const rs2::video_frame(*)>(_a[2])));
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
        if (_id < 52)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 52;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 52)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 52;
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
