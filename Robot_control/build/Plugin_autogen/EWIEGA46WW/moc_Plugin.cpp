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
    QByteArrayData data[45];
    char stringdata0[590];
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
QT_MOC_LITERAL(16, 212, 12), // "connectRobot"
QT_MOC_LITERAL(17, 225, 9), // "stopRobot"
QT_MOC_LITERAL(18, 235, 15), // "teachModeToggle"
QT_MOC_LITERAL(19, 251, 10), // "printArray"
QT_MOC_LITERAL(20, 262, 19), // "std::vector<double>"
QT_MOC_LITERAL(21, 282, 7), // "addMove"
QT_MOC_LITERAL(22, 290, 20), // "write_vector_to_file"
QT_MOC_LITERAL(23, 311, 8), // "myVector"
QT_MOC_LITERAL(24, 320, 11), // "std::string"
QT_MOC_LITERAL(25, 332, 8), // "filename"
QT_MOC_LITERAL(26, 341, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(27, 362, 34), // "std::vector<std::vector<doubl..."
QT_MOC_LITERAL(28, 397, 12), // "Take_picture"
QT_MOC_LITERAL(29, 410, 14), // "Analyze_images"
QT_MOC_LITERAL(30, 425, 27), // "eulerAnglesToRotationMatrix"
QT_MOC_LITERAL(31, 453, 7), // "cv::Mat"
QT_MOC_LITERAL(32, 461, 10), // "cv::Vec3d&"
QT_MOC_LITERAL(33, 472, 5), // "theta"
QT_MOC_LITERAL(34, 478, 7), // "rad2deg"
QT_MOC_LITERAL(35, 486, 6), // "radian"
QT_MOC_LITERAL(36, 493, 7), // "deg2rad"
QT_MOC_LITERAL(37, 501, 6), // "degree"
QT_MOC_LITERAL(38, 508, 16), // "isRotationMatrix"
QT_MOC_LITERAL(39, 525, 8), // "cv::Mat&"
QT_MOC_LITERAL(40, 534, 1), // "R"
QT_MOC_LITERAL(41, 536, 27), // "rotationMatrixToEulerAngles"
QT_MOC_LITERAL(42, 564, 9), // "cv::Vec3d"
QT_MOC_LITERAL(43, 574, 13), // "ReverseVector"
QT_MOC_LITERAL(44, 588, 1) // "v"

    },
    "Plugin\0clickEvent\0\0stateChangedListener\0"
    "rw::kinematics::State\0state\0RunRobotMimic\0"
    "RunRobotControl\0RunHomeRobot\0printQ\0"
    "printTCP\0MoveInToolSpace\0TransformationRobot\0"
    "startRobotMimic\0startRobotControl\0"
    "startHomeRobot\0connectRobot\0stopRobot\0"
    "teachModeToggle\0printArray\0"
    "std::vector<double>\0addMove\0"
    "write_vector_to_file\0myVector\0std::string\0"
    "filename\0createPathRRTConnect\0"
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
      27,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  149,    2, 0x08 /* Private */,
       3,    1,  150,    2, 0x08 /* Private */,
       6,    0,  153,    2, 0x08 /* Private */,
       7,    0,  154,    2, 0x08 /* Private */,
       8,    0,  155,    2, 0x08 /* Private */,
       9,    0,  156,    2, 0x08 /* Private */,
      10,    0,  157,    2, 0x08 /* Private */,
      11,    0,  158,    2, 0x08 /* Private */,
      12,    0,  159,    2, 0x08 /* Private */,
      13,    0,  160,    2, 0x08 /* Private */,
      14,    0,  161,    2, 0x08 /* Private */,
      15,    0,  162,    2, 0x08 /* Private */,
      16,    0,  163,    2, 0x08 /* Private */,
      17,    0,  164,    2, 0x08 /* Private */,
      18,    0,  165,    2, 0x08 /* Private */,
      19,    1,  166,    2, 0x08 /* Private */,
      21,    3,  169,    2, 0x08 /* Private */,
      22,    2,  176,    2, 0x08 /* Private */,
      26,    5,  181,    2, 0x08 /* Private */,
      28,    0,  192,    2, 0x08 /* Private */,
      29,    0,  193,    2, 0x08 /* Private */,
      30,    1,  194,    2, 0x08 /* Private */,
      34,    1,  197,    2, 0x08 /* Private */,
      36,    1,  200,    2, 0x08 /* Private */,
      38,    1,  203,    2, 0x08 /* Private */,
      41,    1,  206,    2, 0x08 /* Private */,
      43,    1,  209,    2, 0x08 /* Private */,

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
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 20,    2,
    0x80000000 | 20, 0x80000000 | 20, QMetaType::Double, QMetaType::Double,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 20, 0x80000000 | 24,   23,   25,
    QMetaType::Void, 0x80000000 | 20, 0x80000000 | 20, QMetaType::Double, 0x80000000 | 27, 0x80000000 | 4,    2,    2,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 31, 0x80000000 | 32,   33,
    QMetaType::Double, QMetaType::Double,   35,
    QMetaType::Double, QMetaType::Double,   37,
    QMetaType::Bool, 0x80000000 | 39,   40,
    0x80000000 | 42, 0x80000000 | 39,   40,
    0x80000000 | 31, 0x80000000 | 39,   44,

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
        case 12: _t->connectRobot(); break;
        case 13: _t->stopRobot(); break;
        case 14: _t->teachModeToggle(); break;
        case 15: _t->printArray((*reinterpret_cast< std::vector<double>(*)>(_a[1]))); break;
        case 16: { std::vector<double> _r = _t->addMove((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< std::vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 17: _t->write_vector_to_file((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 18: _t->createPathRRTConnect((*reinterpret_cast< std::vector<double>(*)>(_a[1])),(*reinterpret_cast< std::vector<double>(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< std::vector<std::vector<double> >(*)>(_a[4])),(*reinterpret_cast< rw::kinematics::State(*)>(_a[5]))); break;
        case 19: _t->Take_picture(); break;
        case 20: _t->Analyze_images(); break;
        case 21: { cv::Mat _r = _t->eulerAnglesToRotationMatrix((*reinterpret_cast< cv::Vec3d(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 22: { double _r = _t->rad2deg((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 23: { double _r = _t->deg2rad((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 24: { bool _r = _t->isRotationMatrix((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 25: { cv::Vec3d _r = _t->rotationMatrixToEulerAngles((*reinterpret_cast< cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Vec3d*>(_a[0]) = std::move(_r); }  break;
        case 26: { cv::Mat _r = _t->ReverseVector((*reinterpret_cast< cv::Mat(*)>(_a[1])));
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
        if (_id < 27)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 27;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 27)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 27;
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
