#ifndef ROBOSLAM_REALSENSE
#define ROBOSLAM_REALSENSE
#include <math.h>
using namespace std;

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <pcl/filters/passthrough.h>

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

/*
 * Realsense camera rotation estimation using ACCELL and GYRO HW components
 */
class rotation_estimator
{
    float3                  theta;                                  // the angle of camera rotation in x, y and z components
    float3                  first_theta;
    std::mutex              theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha             = 0.98;
    bool first              = true;
    double last_ts_gyro     = 0;                                    // Keeps the arrival time of previous gyro frame
public:
    
    void                                process_gyro        (rs2_vector gyro_data, double ts);      // calculate the change in angle of motion based on data from gyro
    void                                process_accel       (rs2_vector accel_data);
    float3                              get_theta           ();                                     // Returns the current rotation angle
    Eigen::Matrix3f                     get_rotation_matrix (float3);
    void                                process             (rs2::motion_frame motion);
};
//=========================================================================================================================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr     points_to_pclxyz    (const rs2::points& points);
std::tuple<int, int, int>               RGB_Texture         (rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  video_frame_2_pcl   (const rs2::points& points, const rs2::video_frame& color);

bool check_imu_is_supported();

#endif