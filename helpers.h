#ifndef CV_HELPERS
#define CV_HELPERS

#include "opencv2/opencv.hpp"
using namespace cv;

#include <librealsense2/rs.hpp>                 // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>   // Load an advanced camera configuraiton
#include <boost/filesystem.hpp>                 // Create directories to store the data
#include <math.h>

#include <Eigen/Dense>

using namespace rs2;
using namespace std;

// MACROS to notify about device events using a sound
#define BEEP_ON  { int out_sys = system("canberra-gtk-play -f /files/Projects/UnderDev/roboslam/libraries/media/start_sound.ogg"); }
#define BEEP_OFF { int out_sys = system("canberra-gtk-play -f /files/Projects/UnderDev/roboslam/libraries/media/prompt.ogg"); }

// Platform independent way to create a directory, if it does not exist
// ref: https://stackoverflow.com/questions/9235679/create-a-directory-if-it-doesnt-exist
void create_dir_if_not_exists(std::string directory);

// Retrieve the device name
std::string get_sensor_name(const rs2::sensor& sensor);
// Get the depth sensor scale
float get_depth_scale(rs2::device dev);
// Check if the device supports an IMU
bool check_imu_is_supported();

struct frmGyro {
public:
    frmGyro(){};
    frmGyro(double ts, rs2_vector m)
    : _ts(ts), _m(m) {};

    double _ts;
    rs2_vector _m;
};
struct frmAcc {
public:
    frmAcc(){};
    frmAcc(double ts, rs2_vector m)
    : _ts(ts), _m(m) {};
    
    double _ts;
    rs2_vector _m;
};
struct frmRGB {
public:
    frmRGB(){};
    frmRGB(double ts, cv::Mat m)
    : _ts(ts), _m(m.clone()) {

    };
    
    double _ts;
    cv::Mat _m;
};
struct frmDepth {
public:
    frmDepth(){};
    frmDepth(double ts, cv::Mat m)
    : _ts(ts), _m(m.clone()){};
    
    double _ts;
    cv::Mat _m;
};

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
struct float3 { 
    float x, y, z; 
    float3  operator*   (float t);
    float3  operator-   (float t);
    float3  operator-   (float3 t);
    void    operator*=  (float t);
    void    operator=   (float3 other);
    void    add         (float t1, float t2, float t3);
};
struct float2 { float x, y; };

struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const;
};

#endif