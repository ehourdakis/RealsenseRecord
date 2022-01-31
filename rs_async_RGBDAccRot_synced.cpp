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
// This is a recorder that stores frames as RGB Depth + multiple IMU acc,rot readings vector
// It captures the frames from the raw callback of the device and syncs them crudely on the software
// without ckecking for keyframes. It uses the boost timing for timestamping.

double dataset_size = 500;

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

std::ofstream rgb_file  ("../rgb.txt",      std::ios_base::out); //std::ios_base::app |
std::ofstream depth_file("../depth.txt",    std::ios_base::out);
std::ofstream imu_file  ("../imu.txt",      std::ios_base::out);

void saveFrameTUMFormatRGBDAccsGyros(RGBDAccRotPair pair, int iframe) {
	char namergb[256]; sprintf(namergb, "../rgb/r%d.png", iframe);
	char namedep[256]; sprintf(namedep, "../depth/d%d.png", iframe);
	char nameimu[256]; sprintf(nameimu, "../imu/i%d.csv", iframe);

    cv::Mat locl_rgb;
    cv::cvtColor  (pair._rgb._m, locl_rgb, cv::COLOR_BGR2RGB);

    cv::imshow("rgb", locl_rgb);
    cv::waitKey(1);
	cv::imwrite(namergb, locl_rgb);
	cv::imwrite(namedep, pair._depth._m);

    std::ofstream pose_prior  (nameimu, std::ios_base::out);
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    
    //Saves the IMU measurements (acceleration(3), rotation(3), timestamp(1))
    for(unsigned i = 0; i < pair._accs.size(); i++ ) {
        Eigen::Matrix<double, 1, 7> accrot;
        accrot << pair._accs[i]._m.x, pair._accs[i]._m.y, pair._accs[i]._m.z, pair._gyros[i]._m.x, pair._gyros[i]._m.y, pair._gyros[i]._m.z, pair._accs[i]._ts;
        pose_prior <<  accrot.format(CSVFormat) << std::endl;
    }
    // pose_prior << rot_prior.format(CSVFormat);
    // std::cout << rot_prior.format(CSVFormat) << std::endl;

    std::cout << "Saving pair: " << iframe << " with " << pair._accs.size() << " accs and " << pair._gyros.size() << " rots " << std::endl;
    depth_file    << pair._depth._ts << " depth/d" << iframe << ".png" << std::endl;
    rgb_file	  << pair._rgb._ts   << " rgb/r"   << iframe << ".png" << std::endl;
    imu_file	  << pair._rgb._ts   << " imu/i"   << iframe << ".csv" << std::endl;
}

// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) 
// try
{
    if(argc > 1) dataset_size = atoi(argv[1]);
    std::cout << "Recording " << dataset_size << " frames" << std::endl;

    //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    //We hold here the timestamped logs
    std::map<int, rs2_vector> gyros;
    std::map<int, rs2_vector> accs;
    std::map<int, cv::Mat> rgbs;
    std::map<int, cv::Mat> depths;

    std::vector<frmAcc>  frame_accs;
    std::vector<frmGyro> frame_gyros;

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
	
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	std::string json_file_name = "/files/Projects/UnderDev/roboslam/documentation/Intel Realsense Cameras/Intel_Configurations/good.json"; //use your own filename here

	std::cout << "Configuring camera : " << serial << std::endl;

	if (dev.is<rs400::advanced_mode>()) {
		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
		// Check if advanced-mode is enabled
		if (!advanced_mode_dev.is_enabled())
		{
			// Enable advanced-mode
			advanced_mode_dev.toggle_advanced_mode(true);
		}
		std::ifstream t(json_file_name, std::ifstream::in);
		std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
		advanced_mode_dev.load_json(preset_json);
		cout << preset_json << endl;
	}
	else {
		std::cout << "Current device doesn't support advanced-mode!\n";
		return EXIT_FAILURE;
	}
	cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_ACCEL, 		RS2_FORMAT_MOTION_XYZ32F, 250);
    cfg.enable_stream(RS2_STREAM_GYRO, 		    RS2_FORMAT_MOTION_XYZ32F, 400);
    cfg.enable_stream(RS2_STREAM_DEPTH,     640,  360,      RS2_FORMAT_Z16,     90);    
    cfg.enable_stream(RS2_STREAM_COLOR,     640,  360, 	    RS2_FORMAT_RGB8, 	90);
    std::cout << "Starting pipe" << std::endl;
    rs2::align align(RS2_STREAM_COLOR);

    auto start = std::chrono::system_clock::now();

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame)
    {
        if(_All_Recorded_Data.size() > dataset_size) return;

        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            // With callbacks, all synchronized stream will arrive in a single frameset

            //Enable this to see RGB and Depth coming
            // for (const rs2::frame& f : fs) {
            //     counters[f.get_profile().unique_id()]++;
            //     std::cout << stream_names[f.get_profile().unique_id()] << " ";
            // }
            // std::cout << std::endl;

            // fs = fs.apply_filter(align);
            rs2::video_frame color_frame = fs.get_color_frame();//

            //My version of timestamping
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end - start;
            double ts = diff.count();
            // double ts = color_frame.get_timestamp();
            
            float width  = color_frame.get_width();//<rs2::video_frame>().get_width();
            float height = color_frame.get_height();
            // std::cout << "Got Color frame" << " " << width << " " << height << std::endl;

            cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
            // cv::cvtColor  (image, image, cv::COLOR_BGR2RGB);
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

            RGBDAccRotPair pair;        //THIS Rotation pair holds an rgb a depth and vecs of imus
            pair._rgb   = frmRGB(ts, image);
            pair._depth = frmDepth(ts, dimage);
            pair._gyros = frame_gyros;
            pair._accs  = frame_accs;
            _All_Recorded_Data.push_back(pair);
            // std::cout << "Saved Frame: " << _All_Recorded_Data.size() << " Timestamp: " << ts << std::endl;

            // cv::imshow("rgb", image);
            // cv::waitKey(1);
            frame_accs.clear();
            frame_gyros.clear();
        }
        else
        {
            // Stream that bypass synchronization (such as IMU) will produce single frames
            counters[frame.get_profile().unique_id()]++;
            //Enable this to see IMUs coming
            // std::cout << stream_names[frame.get_profile().unique_id()] << " " << std::endl;

            rs2::motion_frame motion = frame.as<rs2::motion_frame>();
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get the timestamp of the current frame
                auto end = std::chrono::system_clock::now();
                std::chrono::duration<double> diff = end - start;
                double ts = diff.count();
                // double ts = motion.get_timestamp();
                // std::cout << std::fixed << "GYRO TS: \t" << motion.get_timestamp() << std::endl;

                // Get gyro measures
                rs2_vector gyro_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                // process_gyro(gyro_data, ts);
                gyros[ts] = gyro_data;

                last_gyro = frmGyro(ts, gyro_data);
                // std::cout << "Reading gyro" << std::endl;
            }
            // If casting succeeded and the arrived frame is from accelerometer stream
            //The gyro stream is faster, so we insert a pair acc, gyro when the acc frame comes (using the last gyro measured)
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get accelerometer measures
                auto end = std::chrono::system_clock::now();
                std::chrono::duration<double> diff = end - start;
                double ts = diff.count();
                // double ts = motion.get_timestamp();
                std::cout << std::fixed << "ACCEL TS: \t" << motion.get_timestamp() << std::endl;

                rs2_vector accel_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                accs[ts] = accel_data;
                frame_accs.push_back(frmAcc(ts, accel_data));
                frame_gyros.push_back(last_gyro);

                // std::cout << "Reading accel" << std::endl;
            }
        }
    };

    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    // rs2::pipeline pipe;

    // Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    //

    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    

    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();

    std::cout << "RealSense callback sample" << std::endl << std::endl;

    while (_All_Recorded_Data.size() < dataset_size)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::lock_guard<std::mutex> lock(mutex);

        // std::cout << "\r";
        // for (auto p : counters)
        // {
        //     std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || ";
        // }

        // if(_All_Recorded_Data.size() > DATASET_SIZE) break; 
        // std::cout << "Size: " << _All_Recorded_Data.size() << std::endl;
        
    }
    // pipe.stop();
    std::cout <<"Saving " << _All_Recorded_Data.size() << " frames." << std::endl;          
    for(unsigned i = 0; i < _All_Recorded_Data.size(); i++ ) 
        saveFrameTUMFormatRGBDAccsGyros(_All_Recorded_Data[i], i);   
    pipe.stop();
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