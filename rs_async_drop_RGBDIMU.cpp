// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <fstream>
#include "CV_Helpers.h"

#include <librealsense2/rs.hpp>  
#include <librealsense2/rs_advanced_mode.hpp> 
#include "roboslam_realsense.h"
using namespace rs2;
//This is a recorder that stores frames as RGB Depth + multiple IMU acc,rot readings vector
//It uses the device raw callback and drops the frames with the keyframes as is with their timetamps
//(they need synchronization)

// #define BEEP_ON  { int out_sys = system("canberra-gtk-play -f /files/Projects/UnderDev/roboslam/libraries/media/service-login.ogg"); }
// #define BEEP_OFF { int out_sys = system("canberra-gtk-play -f /files/Projects/UnderDev/roboslam/libraries/media/desktop-login.ogg"); }
#define BEEP_ON  { int out_sys = system("canberra-gtk-play -f /files/Projects/UnderDev/roboslam/libraries/media/start_sound.ogg"); }
#define BEEP_OFF { int out_sys = system("canberra-gtk-play -f /files/Projects/UnderDev/roboslam/libraries/media/prompt.ogg"); }

//IMPORTANT: To get the TIMESTAMPs in realsense set the CMake customization flag ENFORCE_METADATA to 
//true when building the realsense sdk. The value is set to False by default.
int dataset_size = 500;
int recording_mode_fps = 90;

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

//This contains a vector of accs and gyros with the same number of elements
//i.e. we record a gyro when a new accel is available (to get a gyro acc pair)
//see below
class RGBDAccRotPair {
public:
    frmRGB          _rgb;
    frmDepth        _depth;
    std::vector<frmAcc>  _accs;
    std::vector<frmGyro> _gyros;
};

vector<RGBDAccRotPair> _All_Recorded_Data;



static std::string get_sensor_name(const rs2::sensor& sensor)
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

// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) 
// try
{
    std::string rec_dir;
    if(argc > 1) rec_dir            = std::string(argv[1]); else rec_dir = "/files/Projects/UnderDev/roboslam/data/Recorded/";
    if(argc > 2) recording_mode_fps = atoi(argv[2]);        else recording_mode_fps = 90;    //Use 30 for 30 1240x780 fps, 60 for 60 fps and 90 for 90 fps
    if(argc > 3) dataset_size       = atoi(argv[3]);        else dataset_size = 800;
    std::cout << "Recording " << dataset_size << " frames in " << rec_dir << std::endl;

    std::ofstream rgb_file      (rec_dir + std::string("rgb.txt"),      std::ios_base::out); //std::ios_base::app |
    std::ofstream depth_file    (rec_dir + std::string("depth.txt"),    std::ios_base::out);
    std::ofstream acc_file      (rec_dir + std::string("acc.txt"),      std::ios_base::out);
    std::ofstream gyr_file      (rec_dir + std::string("gyr.txt"),      std::ios_base::out);

    //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    //We hold here the timestamped logs
    std::map<double, rs2_vector>        gyrs;
    std::map<double, rs2_vector>        accs;
    std::map<double, cv::Mat>           rgbs;
    std::map<double, cv::Mat>           rgb_intrisics;
    std::map<double, cv::Mat>           depths;

    // std::vector<frmAcc>                 frame_accs;
    // std::vector<frmGyro>                frame_gyros;

    frmGyro last_gyro;

    std::map<int, int> counters;
    std::map<int, std::string> stream_names;
    std::mutex mutex;

    if (!check_imu_is_supported()) {
        std::cerr << "Device supporting IMU not found";
        return EXIT_FAILURE;
    }

    rs2::config 				  		  	cfg;
    rs2::colorizer                          color_map;
	rs2::context ctx;
	auto devices = ctx.query_devices();
	rs2::device dev = devices[0];

	rs2::pipeline pipe(ctx);
    rs2::pipeline_profile profiles;

    //Disable the auto exposure to get the true framerate of the device
    //see this link: https://github.com/IntelRealSense/librealsense/issues/4480#issuecomment-514055336
    //However one general GOOD ADVICE is to sample data when there is plenty of light going on
    std::cout << "Active Sensors: " << "(0) " << get_sensor_name(dev.query_sensors()[0]) << " (1) " << get_sensor_name(dev.query_sensors()[1]) << " (2) " << get_sensor_name(dev.query_sensors()[2]) << std::endl;
    
    dev.query_sensors()[0].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    dev.query_sensors()[1].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    // dev.query_sensors()[0].set_option(rs2_option::RS2_OPTION_EXPOSURE, 33000);
    // dev.query_sensors()[1].set_option(rs2_option::RS2_OPTION_EXPOSURE, 100);
    std::cout << "Sensor " << get_sensor_name(dev.query_sensors()[0]) << " exposure: " << dev.query_sensors()[0].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
    std::cout << "Sensor " << get_sensor_name(dev.query_sensors()[1]) << " exposure: " << dev.query_sensors()[1].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
	
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	// std::string json_file_name = "/files/Projects/UnderDev/roboslam/documentation/Intel Realsense Cameras/Intel_Configurations/good.json"; //use your own filename here
    // std::string json_file_name = "/files/Projects/UnderDev/roboslam/configs/realsense_jsons/high_acc_ritas_room.json"; //use your own filename here
    std::string json_file_name = "/files/Projects/UnderDev/roboslam/configs/realsense_jsons/homeoffice2_december.json";
    std::cout << "Configuring camera : " << serial << std::endl;

	// if (dev.is<rs400::advanced_mode>()) {
	// 	auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
	// 	// Check if advanced-mode is enabled
	// 	if (!advanced_mode_dev.is_enabled())
	// 	{
	// 		// Enable advanced-mode
	// 		advanced_mode_dev.toggle_advanced_mode(true);
	// 	}
	// 	std::ifstream t(json_file_name, std::ifstream::in);
	// 	std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	// 	advanced_mode_dev.load_json(preset_json);
	// 	cout << preset_json << endl;
	// }
	// else {
	// 	std::cout << "Current device doesn't support advanced-mode!\n";
	// 	return EXIT_FAILURE;
	// }
	cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_ACCEL, 		            RS2_FORMAT_MOTION_XYZ32F,   250);
    cfg.enable_stream(RS2_STREAM_GYRO, 		                RS2_FORMAT_MOTION_XYZ32F,   400);

    if(recording_mode_fps==30){
        cfg.enable_stream(RS2_STREAM_DEPTH,     1280,  720,      RS2_FORMAT_Z16,            30);    
        cfg.enable_stream(RS2_STREAM_COLOR,     1280,  720,      RS2_FORMAT_RGB8, 	        30);
        std::cout << "Recording 1280x720 @ 30fps" << std::endl;
    }
    if(recording_mode_fps==60){
        cfg.enable_stream(RS2_STREAM_DEPTH,     848,  480,       RS2_FORMAT_Z16,            60);    
        cfg.enable_stream(RS2_STREAM_COLOR,     848,  480,       RS2_FORMAT_RGB8, 	        60);
        std::cout << "Recording 848x480 @ 60fps" << std::endl;
    }
    if(recording_mode_fps==90){
        cfg.enable_stream(RS2_STREAM_DEPTH,     640,  360,      RS2_FORMAT_Z16,             90);    
        cfg.enable_stream(RS2_STREAM_COLOR,     640,  360, 	    RS2_FORMAT_RGB8, 	        90);
        std::cout << "Recording 640x360 @ 90fps" << std::endl;
    }
    rs2::align align(RS2_STREAM_DEPTH);         // Align all frames to depth viewport
    // rs2::align align(RS2_STREAM_COLOR);         // Align all frames to color viewport
    
    auto start = std::chrono::system_clock::now();

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    bool bfirst = false;
    auto callback = [&](const rs2::frame& frame)
    {

        if(depths.size() > dataset_size) return;

        std::lock_guard<std::mutex> lock(mutex);

        double wait_time = 1.0;
        if(!bfirst) {
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end - start;
            double ts = diff.count();
            std::cout << "Starting in " << std::setprecision(2) << wait_time - ts << " s" << "\r";
            // std::this_thread::sleep_for(std::chrono::seconds(5)); bfirst=false;
            if(ts >= wait_time) {bfirst = true; std::cout << "Started Recording               " << "\n"; BEEP_ON;}
            return;
        };

        std::cout << "Recording " << (int)dataset_size << " frames " << "Depths: " << depths.size() << " RGBs: " << rgbs.size() << " Accs " << accs.size() << " Gyrs " << gyrs.size() << "\r";//std::endl; 
        
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {

            // fs = align.process(fs); 
            // fs = fs.apply_filter(align);           
            // Enable this to see RGB and Depth coming
            // for (const rs2::frame& f : fs) {
            //     counters[f.get_profile().unique_id()]++;
            //     std::cout << stream_names[f.get_profile().unique_id()] << " ";
            // }
            // std::cout << " " << rgbs.size() <<  " " << depths.size() << std::endl;

            rs2::video_frame color_frame = fs.get_color_frame().apply_filter(align);
            float width  = color_frame.get_width();
            float height = color_frame.get_height();
            cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
            // cv::cvtColor  (image, image, cv::COLOR_BGR2RGB);
            rgbs[color_frame.get_timestamp()] = image.clone();

            // // Per frame intrisics dont seem to work
            // //auto aaa = fs.get_color_frame().get_profile().as<rs2::video_stream_profile>();
            // auto aaa = color_frame.get_profile().as<rs2::video_stream_profile>();;
            // //auto color_stream = profiles.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            // //auto resolution = std::make_pair(color_stream.width(), color_stream.height());
            // auto i = aaa.get_intrinsics();
            // auto principal_point = std::make_pair(i.ppx, i.ppy);
            // auto focal_length = std::make_pair(i.fx, i.fy);
            // rs2_distortion model = i.model;
            // std::cout << std::fixed << std::setprecision(8) << "Intrisics: " << "px: " << i.ppx << " py: " << i.ppy << " fx: " << i.fx << " fy: " << i.fy << std::endl;


            // cv::imshow("rgb", image);
            // cv::waitKey(1);
            //rs2::frame rdepth = color_map(data.get_depth_frame()); // Find and colorize the depth data
            // std::cout << "Got Color frame" << " " << std::fixed  << color_frame.get_timestamp() << std::endl;

            rs2::video_frame depth_frame = fs.get_depth_frame();//.apply_filter(align);//.apply_filter(color_map);
            float dwidth  = depth_frame.get_width();
            float dheight = depth_frame.get_height();        
            cv::Mat dimage(cv::Size(dwidth, dheight), CV_16UC1, (void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
            depths[depth_frame.get_timestamp()] = dimage.clone();
            // std::cout << "Got Depth frame" << " " << std::fixed << depth_frame.get_timestamp() << std::endl;

            // RGBDAccRotPair pair;        //THIS Rotation pair holds an rgb a depth and vecs of imus
            // pair._rgb   = frmRGB    (color_frame.get_timestamp(), image);
            // pair._depth = frmDepth  (depth_frame.get_timestamp(), dimage);
            // pair._gyros = frame_gyros;
            // pair._accs  = frame_accs;
            // _All_Recorded_Data.push_back(pair);
            // std::cout << "Saved Frame: " << _All_Recorded_Data.size() << " Timestamp: " << ts << std::endl;

            // cv::imshow("rgb", image);
            // cv::waitKey(1);
            // frame_accs.clear();
            // frame_gyros.clear();
        }
        else
        {
            static float3 theta;
            static double last_ts_gyro = -1000.0;
            static float3 gyro_d;
            // Stream that bypass synchronization (such as IMU) will produce single frames
            counters[frame.get_profile().unique_id()]++;
            //Enable this to see IMUs coming
            // std::cout << stream_names[frame.get_profile().unique_id()] << " " << std::endl;
            rs2::motion_frame motion = frame.as<rs2::motion_frame>();
            if(last_ts_gyro==-1000.0) last_ts_gyro = motion.get_timestamp();
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                double ts = motion.get_timestamp();
                // std::cout << std::fixed << "GYRO TS: \t" << motion.get_timestamp() << std::endl;
                rs2_vector gyro_data = motion.get_motion_data();
                gyrs[ts] = gyro_data;
                // last_gyro = frmGyro(ts, gyro_data);

#ifdef COMPUTE_GYRO
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
        // std::lock_guard<std::mutex> lock(theta_mtx);
        // theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);

        theta.add(gyro_angle.x, gyro_angle.y, gyro_angle.z);
        std::cout << "Reading gyro  " << gyro_data << " theta: " << theta.x << " " << theta.y << " " << theta.z << std::endl;
#endif
            }
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                double ts = motion.get_timestamp();
                // std::cout << std::fixed << "ACCEL TS: \t" << motion.get_timestamp() << std::endl;
                rs2_vector accel_data = motion.get_motion_data();
                accs[ts] = accel_data;
                //std::cout << accel_data << std::endl;
                // frame_accs.push_back(frmAcc(ts, accel_data));
                // frame_gyros.push_back(last_gyro);
            }
        }
    };

    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    // rs2::pipeline pipe;

    std::cout << "Starting pipe" << std::endl;
    // Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    profiles = pipe.start(cfg, callback);
    float depth_scale = get_depth_scale(profiles.get_device());
    std::cout << "Scale is: " << depth_scale << std::endl;


    auto color_stream = profiles.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto resolution = std::make_pair(color_stream.width(), color_stream.height());
    auto intr = color_stream.get_intrinsics();
    auto principal_point = std::make_pair(intr.ppx, intr.ppy);
    auto focal_length = std::make_pair(intr.fx, intr.fy);
    rs2_distortion model = intr.model;
    std::cout << std::fixed << std::setprecision(10) << "Intrisics: " << "px: " << intr.ppx << " py: " << intr.ppy << " fx: " << intr.fx << " fy: " << intr.fy << std::endl;
    std::ofstream fintrinsics  (rec_dir + "rgb.intrisics", std::ios_base::out);
    fintrinsics << std::setprecision(10) << intr.fx      << ", 0.0, "    << intr.ppx << std::endl;
    fintrinsics << std::setprecision(10) << "0.0, "      << intr.fy      << ", " << intr.ppy << std::endl;
    fintrinsics << std::setprecision(10) << "0.0, "      << "0.0, "      << "1.0" << std::endl;
    
    
    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();

    std::cout << "RealSense get frames from device callback (Unsynchronized)" << std::endl;

    while (depths.size() <= dataset_size)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // std::lock_guard<std::mutex> lock(mutex);
        // std::cout << "RGBs: " << rgbs.size() << " Depths: " << depths.size() << " Accs " << accs.size() << " Gyrs " << gyrs.size() << "\n"; //\r 
        // for (auto p : counters)
        // {
        //     std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || ";
        // }

        // if(_All_Recorded_Data.size() > DATASET_SIZE) break; 
        // std::cout << "Size: " << _All_Recorded_Data.size() << std::endl;
        
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {
        //Lock the pipe stopping to make sure that no data are messed in the callback
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << std::endl << "Stopping device" << std::endl;
        pipe.stop();
        std::cout << "Stopped" << std::endl;
        BEEP_OFF;
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));         
    std::cout << std::fixed;
    // for (auto i : rgbs)     cout << "Got RGB "    << i.first << endl;
    // for (auto i : depths)   cout << "Got Depth "  << i.first << endl;
    // for (auto i : accs)     cout << "Got Acc "    << i.first << endl;
    // for (auto i : gyrs)     cout << "Got Gyro "   << i.first << endl;
    
    // depth_file    << pair._depth._ts << " depth/d" << iframe << ".png" << std::endl;
    // imu_file	  << pair._rgb._ts   << " imu/i"   << iframe << ".csv" << std::endl;
    // std::lock_guard<std::mutex> lock(mutex);
    std::cout << "Playing back " << rgbs.size() << " frames" << std::endl;
    int index = 0;
    for(auto in : rgbs) {
        cv::Mat locl_rgb;
        cv::cvtColor  (in.second, locl_rgb, cv::COLOR_BGR2RGB);
        cv::imshow("rgb", locl_rgb);
        std::cout << "Frame " << index++ << "\r";
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "\nSaving " << rgbs.size() << " RGBs" << "\n";
    int ii = 0;
    for (auto i : rgbs) {
        std::string namergb = std::string("rgb/r") + to_string(ii++) + std::string(".png"); 
        rgb_file << std::fixed << i.first << " " << namergb << std::endl;
        cv::Mat locl_rgb;
        cv::cvtColor  (i.second, locl_rgb, cv::COLOR_BGR2RGB);
        cv::imwrite(rec_dir + namergb, locl_rgb);
        std::cout << "Saved " << ii << "\r";
    }
    std::cout << "\nSaving " << depths.size() << " Depths" << "\n";
    int dd = 0;
    for (auto i : depths) {
        std::string namedepth = std::string("depth/d") + to_string(dd++) + std::string(".png"); 
        depth_file << std::fixed << i.first << " " << namedepth << std::endl;
        // cv::imshow("depth", depths[i.first]);
        // cv::waitKey(1);
        cv::imwrite(rec_dir + namedepth, depths[i.first]);
        std::cout << "Saved " << dd << "\r";
    }
    std::cout << "\nSaving " << accs.size() << " Accellerations" << "\n";
    int aa = 0;
    for (auto i : accs) {
        std::string nameacc = std::string("acc/a") + to_string(aa++) + std::string(".txt"); 
        acc_file << std::fixed << i.first << " " << nameacc << std::endl;
        std::ofstream acc_  (rec_dir + nameacc, std::ios_base::out);
        acc_ << accs[i.first].x << " " <<  accs[i.first].y << " " << accs[i.first].z << std::endl;
        std::cout << "Saved " << aa << "\r";
    }
    int gg = 0;
    std::cout << "\nSaving " << gyrs.size() << " Gyros" << "\n";
    for (auto i : gyrs) {
        std::string namegyr = std::string("gyr/g") + to_string(gg++) + std::string(".txt"); 
        gyr_file << std::fixed << i.first << " " << namegyr << std::endl;
        std::ofstream gyr_  (rec_dir + namegyr, std::ios_base::out);
        gyr_ << gyrs[i.first].x << " " <<  gyrs[i.first].y << " " << gyrs[i.first].z << std::endl;
        std::cout << "Saved " << gg << "\r";
    }
    std::cout << std::endl;

    // for(unsigned i = 0; i < _All_Recorded_Data.size(); i++ ) 
    //     saveFrameTUMFormatRGBDAccsGyros(_All_Recorded_Data[i], i);   
    // pipe.stop();
     return EXIT_SUCCESS;
}
// catch (const rs2::error & e)
// {
//     std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//     return EXIT_FAILURE;
// }
// catch (const std::exception& e)
// {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }