#include "helpers.h"

void create_dir_if_not_exists(std::string directory) {
    const char* path = directory.c_str();
    boost::filesystem::path dir(path);
    if(boost::filesystem::create_directory(dir))
    {
        std::cerr<< "Directory Created: "<< directory <<std::endl;
    }
}

std::string get_sensor_name(const rs2::sensor& sensor)
{
    // Sensors support additional information, such as a human readable name
    if (sensor.supports(RS2_CAMERA_INFO_NAME))
        return sensor.get_info(RS2_CAMERA_INFO_NAME);
    else
        return "Unknown Sensor";
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
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