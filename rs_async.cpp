#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include "CV_Helpers.h"

//This is a recorder that runs an asynchronous callback to get all the frames for IMU (accel, rot) and RGB, Depth streams. 
//Streams are marked with a timestamp. 

int main(int argc, char * argv[]) try
{
    //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    //We hold here the timestamped logs
    std::map<int, rs2_vector> gyros;
    std::map<int, rs2_vector> accs;
    std::map<int, cv::Mat> rgbs;
    std::map<int, cv::Mat> depths;

    std::map<int, int> counters;
    std::map<int, std::string> stream_names;
    std::mutex mutex;

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            // With callbacks, all synchronized stream will arrive in a single frameset
            for (const rs2::frame& f : fs) {
                counters[f.get_profile().unique_id()]++;
                std::cout << stream_names[f.get_profile().unique_id()] << " ";
            }
            std::cout << std::endl;

            rs2::video_frame color_frame = fs.get_color_frame();//

            double ts = color_frame.get_timestamp();
            float width  = color_frame.get_width();//<rs2::video_frame>().get_width();
            float height = color_frame.get_height();
            // std::cout << "Got Color frame" << " " << width << " " << height << std::endl;

            cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
            cv::cvtColor  (image, image, cv::COLOR_BGR2RGB);
            rgbs[ts] = image;
            // char rname[256]; sprintf(rname, "rgb/frame%.3d.png", frame_num);
            // std::cout << rname << std::endl;
            // cv::imwrite(rname, image);        
            //rs2::frame rdepth = color_map(data.get_depth_frame()); // Find and colorize the depth data
            rs2::video_frame depth_frame = fs.get_depth_frame();//.apply_filter(color_map);
            float dwidth  = depth_frame.get_width();//<rs2::video_frame>().get_width();
            float dheight = depth_frame.get_height();        
            // std::cout << "Got Depth frame" << " " << dwidth << " " << dheight << std::endl;
            cv::Mat dimage(cv::Size(dwidth, dheight), CV_16UC1, (void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
            depths[ts] = dimage;
        }
        else
        {
            // Stream that bypass synchronization (such as IMU) will produce single frames
            counters[frame.get_profile().unique_id()]++;
            std::cout << stream_names[frame.get_profile().unique_id()] << " " << std::endl;

            rs2::motion_frame motion = frame.as<rs2::motion_frame>();
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get the timestamp of the current frame
                double ts = motion.get_timestamp();
                // Get gyro measures
                rs2_vector gyro_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                // process_gyro(gyro_data, ts);
                gyros[ts] = gyro_data;
                // std::cout << "Reading gyro" << std::endl;
            }
            // If casting succeeded and the arrived frame is from accelerometer stream
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get accelerometer measures
                double ts = motion.get_timestamp();
                rs2_vector accel_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                accs[ts] = accel_data;
                // std::cout << "Reading accel" << std::endl;
            }
        }
    };

    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    rs2::pipeline pipe;

    // Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    //
    rs2::pipeline_profile profiles = pipe.start(callback);

    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();

    std::cout << "RealSense callback sample" << std::endl << std::endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::lock_guard<std::mutex> lock(mutex);

        std::cout << "\r";
        for (auto p : counters)
        {
            std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || ";
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}