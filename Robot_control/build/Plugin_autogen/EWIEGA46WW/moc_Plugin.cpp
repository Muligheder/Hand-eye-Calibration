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
    QByteArrayData data[52];
    char stringdata0[692];
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
QT_MOC_LITERAL(15, 197, 22), // "startRobotControlRoute"
QT_MOC_LITERAL(16, 220, 14), // "startHomeRobot"
QT_MOC_LITERAL(17, 235, 6), // "invKin"
QT_MOC_LITERAL(18, 242, 19), // "std::vector<double>"
QT_MOC_LITERAL(19, 262, 12), // "connectRobot"
QT_MOC_LITERAL(20, 275, 9), // "stopRobot"
QT_MOC_LITERAL(21, 285, 8), // "stopSync"
QT_MOC_LITERAL(22, 294, 15), // "teachModeToggle"
QT_MOC_LITERAL(23, 310, 15), // "getConfDistance"
QT_MOC_LITERAL(24, 326, 10), // "printArray"
QT_MOC_LITERAL(25, 337, 7), // "addMove"
QT_MOC_LITERAL(26, 345, 20), // "write_vector_to_file"
QT_MOC_LITERAL(27, 366, 8), // "myVector"
QT_MOC_LITERAL(28, 375, 11), // "std::string"
QT_MOC_LITERAL(29, 387, 8), // "filename"
QT_MOC_LITERAL(30, 396, 16), // "printDeviceNames"
QT_MOC_LITERAL(31, 413, 20), // "rw::models::WorkCell"
QT_MOC_LITERAL(32, 434, 8), // "workcell"
QT_MOC_LITERAL(33, 443, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(34, 464, 34), // "std::vector<std::vector<doubl..."
QT_MOC_LITERAL(35, 499, 12), // "Take_picture"
QT_MOC_LITERAL(36, 512, 14), // "Analyze_images"
QT_MOC_LITERAL(37, 527, 27), // "eulerAnglesToRotationMatrix"
QT_MOC_LITERAL(38, 555, 7), // "cv::Mat"
QT_MOC_LITERAL(39, 563, 10), // "cv::Vec3d&"
QT_MOC_LITERAL(40, 574, 5), // "theta"
QT_MOC_LITERAL(41, 580, 7), // "rad2deg"
QT_MOC_LITERAL(42, 588, 6), // "radian"
QT_MOC_LITERAL(43, 595, 7), // "deg2rad"
QT_MOC_LITERAL(44, 603, 6), // "degree"
QT_MOC_LITERAL(45, 610, 16), // "isRotationMatrix"
QT_MOC_LITERAL(46, 627, 8), // "cv::Mat&"
QT_MOC_LITERAL(47, 636, 1), // "R"
QT_MOC_LITERAL(48, 638, 27), // "rotationMatrixToEulerAngles"
QT_MOC_LITERAL(49, 666, 9), // "cv::Vec3d"
QT_MOC_LITERAL(50, 676, 13), // "ReverseVector"
QT_MOC_LITERAL(51, 690, 1) // "v"

    },
    "Plugin\0clickEvent\0\0stateChangedListener\0"
    "rw::kinematics::State\0state\0RunRobotMimic\0"
    "RunRobotControl\0RunHomeRobot\0printQ\0"
    "printTCP\0MoveInToolSpace\0TransformationRobot\0"
    "startRobotMimic\0startRobotControl\0"
    "startRobotControlRoute\0startHomeRobot\0"
    "invKin\0std::vector<double>\0connectRobot\0"
    "stopRobot\0stopSync\0teachModeToggle\0"
    "getConfDistance\0printArray\0addMove\0"
    "write_vector_to_file\0myVector\0std::string\0"
    "filename\0printDeviceNames\0"
    "rw::models::WorkCell\0workcell\0"
    "createPathRRTConnect\0"
    "std::vector<std::vector<double> >&\0"
    "Take_picture\0Analyze_images\0"
    "eulerAnglesToRotationMatrix\0cv::Mat\0"
    "cv::Vec3d&\0theta\0rad2deg\0radian\0deg2rad\0"
    "degree\0isRotationMatrix\0cv::Mat&\0R\0"
    "rotationMatrixToEulerAngles\0cv::Vec3d\0"
    "ReverseVector\0v"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Plugin[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      32,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  174,    2, 0x08 /* Private */,
       3,    1,  175,    2, 0x08 /* Private */,
       6,    0,  178,    2, 0x08 /* Private */,
       7,    0,  179,    2, 0x08 /* Private */,
       8,    0,  180,    2, 0x08 /* Private */,
       9,    0,  181,    2, 0x08 /* Private */,
      10,    0,  182,    2, 0x08 /* Private */,
      11,    0,  183,    2, 0x08 /* Private */,
      12,    0,  184,    2, 0x08 /* Private */,
      13,    0,  185,    2, 0x08 /* Private */,
      14,    0,  186,    2, 0x08 /* Private */,
      15,    0,  187,    2, 0x08 /* Private */,
      16,    0,  188,    2, 0x08 /* Private */,
      17,    2,  189,    2, 0x08 /* Private */,
      19,    0,  194,    2, 0x08 /* Private */,
      20,    0,  195,    2, 0x08 /* Private */,
      21,    0,  196,    2, 0x08 /* Private */,
      22,    0,  197,    2, 0x08 /* Private */,
      23,    2,  198,    2, 0x08 /* Private */,
      24,    1,  203,    2, 0x08 /* Private */,
      25,    4,  206,    2, 0x08 /* Private */,
      26,    2,  215,    2, 0x08 /* Private */,
      30,    1,  220,    2, 0x08 /* Private */,
      33,    8,  223,    2, 0x08 /* Private */,
      35,    0,  240,    2, 0x08 /* Private */,
      36,    0,  241,    2, 0x08 /* Private */,
      37,    1,  242,    2, 0x08 /* Private */,
      41,    1,  245,    2, 0x08 /* Private */,
      43,    1,  248,    2, 0x08 /* Private */,
      45,    1,  251,    2, 0x08 /* Private */,
      48,    1,  254,    2, 0x08 /* Private */,
      50,    1,  257,    2, 0x08 /* Private */,

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
    QMetaType::Void,
    0x80000000 | 18, 0x80000000 | 18, 0x80000000 | 18,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Double, 0x80000000 | 18, 0x80000000 | 18,    2,    2,
    QMetaType::Void, 0x80000000 | 18,    2,
    0x80000000 | 18, 0x80000000 | 18, QMetaType::Double, QMetaType::Double, QMetaType::Double,    2,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 18, 0x80000000 | 28,   27,   29,
    QMetaType::Void, 0x80000000 | 31,   32,
    QMetaType::Void, 0x80000000 | 18, 0x80000000 | 18, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, 0x80000000 | 34, 0x80000000 | 4,    2,    2,    2,    2,    2,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 38, 0x80000000 | 39,   40,
    QMetaType::Double, QMetaType::Double,   42,
    QMetaType::Double, QMetaType::Double,   44,
    QMetaType::Bool, 0x80000000 | 46,   47,
    0x80000000 | 49, 0x80000000 | 46,   47,
    0x80000000 | 38, 0x80000000 | 46,   51,

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
        case 11: _t->startRobotControlRoute(); break;
        case 12: _t->startHomeRobot(); break;
        case 13: { std::vector<double> _r = _t->invKin((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 14: _t->connectRobot(); break;
        case 15: _t->stopRobot(); break;
        case 16: _t->stopSync(); break;
        case 17: _t->teachModeToggle(); break;
        case 18: { double _r = _t->getConfDistance((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 19: _t->printArray((*reinterpret_cast< std::vector<double>(*)>(_a[1]))); break;
        case 20: { std::vector<double> _r = _t->addMove((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 21: _t->write_vector_to_file((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 22: _t->printDeviceNames((*reinterpret_cast< const rw::models::WorkCell(*)>(_a[1]))); break;
        case 23: _t->createPathRRTConnect((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< std::vector<std::vector<double> >(*)>(_a[7])),(*reinterpret_cast< rw::kinematics::State(*)>(_a[8]))); break;
        case 24: _t->Take_picture(); break;
        case 25: _t->Analyze_images(); break;
        case 26: { cv::Mat _r = _t->eulerAnglesToRotationMatrix((*reinterpret_cast< cv::Vec3d(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 27: { double _r = _t->rad2deg((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 28: { double _r = _t->deg2rad((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 29: { bool _r = _t->isRotationMatrix((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 30: { cv::Vec3d _r = _t->rotationMatrixToEulerAngles((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Vec3d*>(_a[0]) = std::move(_r); }  break;
        case 31: { cv::Mat _r = _t->ReverseVector((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
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
        if (_id < 32)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 32;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 32)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 32;
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
