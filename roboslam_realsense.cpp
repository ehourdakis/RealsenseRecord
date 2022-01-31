#include "roboslam_realsense.h"

void rotation_estimator::process_gyro(rs2_vector gyro_data, double ts)
{
    if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        last_ts_gyro = ts;
        // first = false;
        return;
    }
    // Holds the change in angle, as calculated from gyro
    float3 gyro_angle;

    // Initialize gyro_angle with data from gyro
    gyro_angle.x = gyro_data.x; // Pitch
    gyro_angle.y = gyro_data.y; // Yaw
    gyro_angle.z = gyro_data.z; // Roll

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;

    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mtx);
    theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    std::cout << "FLoat3 " << -gyro_angle.z<< -gyro_angle.y<< gyro_angle.x << std::endl;
}

void rotation_estimator::process_accel(rs2_vector accel_data)
{
    // Holds the angle as calculated from accelerometer data
    float3 accel_angle;

    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

    // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mtx);
    if (first)
    {
        first = false;
        theta = accel_angle;
        
        // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
        theta.y = PI;

        first_theta = theta;
    }
    else
    {
        /* 
        Apply Complementary Filter:
            - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                that are steady over time, is used to cancel out drift.
            - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
        */
        theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
        theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
    }
}

// Returns the current rotation angle
float3 rotation_estimator::get_theta()
{
    std::lock_guard<std::mutex> lock(theta_mtx);
    return theta - first_theta;
}

Eigen::Matrix3f rotation_estimator::get_rotation_matrix (float3 theta3) {
    // float3 theta3 = get_theta();
    Eigen::Matrix3f ret;
    ret =     Eigen::AngleAxisf(theta3.x, Eigen::Vector3f::UnitX()) //Pitch
            * Eigen::AngleAxisf(theta3.y, Eigen::Vector3f::UnitY()) //Yaw
            * Eigen::AngleAxisf(theta3.z, Eigen::Vector3f::UnitZ());//Rolll
    // ret =     Eigen::AngleAxisf(theta.x, Eigen::Vector3f(1,0,0)) //
    //         * Eigen::AngleAxisf(theta.y, Eigen::Vector3f(0,1,0)) //
    //         * Eigen::AngleAxisf(theta.z, Eigen::Vector3f(0,0,1));//
    return ret;
}

void rotation_estimator::process(rs2::motion_frame motion) {
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        // Get the timestamp of the current frame
        double ts = motion.get_timestamp();
        // Get gyro measures
        rs2_vector gyro_data = motion.get_motion_data();
        // Call function that computes the angle of motion based on the retrieved measures
        process_gyro(gyro_data, ts);
        std::cout << "Reading gyro" << std::endl;
    }
    // If casting succeeded and the arrived frame is from accelerometer stream
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        // Get accelerometer measures
        rs2_vector accel_data = motion.get_motion_data();
        // Call function that computes the angle of motion based on the retrieved measures
        process_accel(accel_data);
        // std::cout << "Reading accel" << std::endl;
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pclxyz(const rs2::points& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr video_frame_2_pcl(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
// pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
// Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
// Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
// Cloud_Filter.setFilterLimits (0.0, 2.0);      // Set accepted interval values
// Cloud_Filter.filter (*cloud);              // Filtered Cloud Outputted


   return cloud; // PCL RGB Point Cloud generated
}
bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}


float3 float3::operator*(float t)
{
    return { x * t, y * t, z * t };
}

float3 float3::operator-(float t)
{
    return { x - t, y - t, z - t };
}


float3 float3::operator-(float3 t) {
    return {x - t.x, y - t.y, z - t.z};
}

void float3::operator*=(float t)
{
    x = x * t;
    y = y * t;
    z = z * t;
}

void float3::operator=(float3 other)
{
    x = other.x;
    y = other.y;
    z = other.z;
}

void float3::add(float t1, float t2, float t3)
{
    x += t1;
    y += t2;
    z += t3;
}

rect rect::adjust_ratio(float2 size) const
{
    auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
    if (W > w)
    {
        auto scale = w / W;
        W *= scale;
        H *= scale;
    }

    return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
}