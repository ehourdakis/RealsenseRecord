#include <stdio.h>
#include <iostream>
using namespace std;
#include <stdio.h>
#include <iostream> 
#include <vector>
#include <fstream>
#include <math.h>
#include <cmath>
using namespace std;

// #include "logging.h"
// INITIALIZE_EASYLOGGINGPP

// #include "example.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include "CV_Helpers.h"
#define USE_DATA
#define TTYPE int32_t

#define BLOCK_FMT 	2, 70, 1
#define FRM_FMT 	0, 80, 9

#include <librealsense2/rs.hpp>  
#include <librealsense2/rs_advanced_mode.hpp> 
#include "roboslam_realsense.h"
using namespace rs2;

//This is a recorded for Realsense that converts IMU measurements to a single rotation matrix on each frame

//You can find the Realsense IMU motion sensor data info here: https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/

int log_verbosity_level = 1;
std::ofstream rgb_file  ("../rgb.txt",      std::ios_base::out); //std::ios_base::app |
std::ofstream depth_file("../depth.txt",    std::ios_base::out);
std::ofstream imu_file  ("../imu.txt",      std::ios_base::out);

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

void saveFrameTUMFormatIMU(cv::Mat &rgb, cv::Mat &depth, Eigen::Matrix3f rot_prior, int iframe) {
	char namergb[256]; sprintf(namergb, "../rgb/r%d.png", iframe);
	char namedep[256]; sprintf(namedep, "../depth/d%d.png", iframe);
	char nameimu[256]; sprintf(nameimu, "../imu/i%d.csv", iframe);
    
	cv::imwrite(namergb, rgb);
	cv::imwrite(namedep, depth);
    std::ofstream pose_prior  (nameimu, std::ios_base::out);
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    pose_prior << rot_prior.format(CSVFormat);
    // std::cout << rot_prior.format(CSVFormat) << std::endl;

    depth_file    << "12312323.12312" << " depth/d" << iframe << ".png" << std::endl;
    rgb_file	  << "12312323.12312" << " rgb/r"   << iframe << ".png" << std::endl;
    imu_file	  << "12312323.12312" << " imu/i"   << iframe << ".csv" << std::endl;
}
void saveFrameTUMFormat(cv::Mat &rgb, cv::Mat &depth, int iframe) {
	char namergb[256]; sprintf(namergb, "../rgb/r%d.png", iframe);
	char namedep[256]; sprintf(namedep, "../depth/d%d.png", iframe);
	cv::imwrite(namergb, rgb);
	cv::imwrite(namedep, depth);

    depth_file    << "12312323.12312" << " depth/d" << iframe << ".png" << std::endl;
    rgb_file	  << "12312323.12312" << " rgb/r"   << iframe << ".png" << std::endl;
}

int main(int argc, const char *argv[]) {
    // CVOpenNIAsus asus;
    // asus.open();
    // int frame = 0;
    // while(true) {
    //     std::cout << "Frame " << frame++ << std::endl;
    //     cv::imshow("r", r); cv::imshow("d", d); cv::waitKey(1);
    // }

    if (!check_imu_is_supported())
    {
        std::cerr << "Device supporting IMU not found";
        return EXIT_FAILURE;
    }

//     configure_logging();

    //   pcl::visualization::PCLVisualizer 	                viewer								("roboslam: Frame info", 			false);

    // viewer.createInteractor								();
    // // viewer.loadCameraParameters		    				(data.datadir() + "/viewer_camera.params");
    // // viewer.getCameraParameters 							(viewer_cam);
    // // viewer.setPosition 			   						(demo_cam.window_size[0], 0);  
    // viewer.setBackgroundColor 			       			(34.0/255.0, 42.0/255.0, 53.0/255.0);
		
    // rs2::pipeline         		  		    pipe; 
    
    rs2::config 				  		  	cfg;
    rs2::colorizer                          color_map;

////////////////////////////////////////////////////////////////////
	rs2::context ctx; //declare context object in main function
	//get device
	auto devices = ctx.query_devices();
	rs2::device dev = devices[0];

	rs2::pipeline pipe(ctx);
	
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	std::string json_file_name = "/files/Projects/UnderDev/roboslam/documentation/Intel Realsense Cameras/Intel_Configurations/good.json"; //use your own filename here

	std::cout << "Configuring camera : " << serial << std::endl;

	if (dev.is<rs400::advanced_mode>())
	{
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
	else
	{
		std::cout << "Current device doesn't support advanced-mode!\n";
		return EXIT_FAILURE;
	}

	cfg.enable_device(serial);
///////////////////////////////////////////////////////////////////////







    cfg.enable_stream(RS2_STREAM_ACCEL, 		RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, 		    RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(RS2_STREAM_POSE, 		  RS2_FORMAT_MOTION_XYZ32F);
// 
    // cfg.enable_stream(RS2_STREAM_DEPTH,     1280, 720,      RS2_FORMAT_Z16,     30);    
    // cfg.enable_stream(RS2_STREAM_COLOR,     1280, 720, 	    RS2_FORMAT_RGB8, 	30);
    // cfg.enable_stream(RS2_STREAM_COLOR);
    // cfg.enable_stream(RS2_STREAM_DEPTH);
    // cfg.enable_stream(RS2_STREAM_DEPTH,     640,  480,      RS2_FORMAT_Z16,     60);    
    // cfg.enable_stream(RS2_STREAM_COLOR,     640,  480, 	    RS2_FORMAT_RGB8, 	60);

    cfg.enable_stream(RS2_STREAM_DEPTH,     640,  360,      RS2_FORMAT_Z16,     90);    
    cfg.enable_stream(RS2_STREAM_COLOR,     640,  360, 	    RS2_FORMAT_RGB8, 	90);
    
    std::cout << "Starting pipe" << std::endl;
    rs2::pipeline_profile profile = pipe.start(cfg);
    float depth_scale = get_depth_scale(profile.get_device());
    std::cout << "Scale is: " << depth_scale << std::endl;
    
    rs2::align align(RS2_STREAM_DEPTH);

    // rs2::decimation_filter dec;
    // dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // rs2::disparity_transform depth2disparity;
    // rs2::disparity_transform disparity2depth(false);
    // rs2::spatial_filter spat;
    // spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    // Define temporal filter
    // rs2::temporal_filter temp;

    rs2::points points;
    rs2::pointcloud pc;
    rotation_estimator algo;

    static int frame_num = 0;
    try {
        while(true) {
        rs2::frameset data = pipe.wait_for_frames();//.apply_filter(color_map);

    // // Find and retrieve IMU and/or tracking data
    // if (rs2::motion_frame accel_frame = data.first_or_default(RS2_STREAM_ACCEL))    {
    //     rs2_vector accel_sample = accel_frame.get_motion_data();
    //     std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
    //     //...
    // }
 
    // if (rs2::motion_frame gyro_frame = data.first_or_default(RS2_STREAM_GYRO))    {
    //     rs2_vector gyro_sample = gyro_frame.get_motion_data();
    //     std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
    //     //...
    // }
 
    // if (rs2::pose_frame pose_frame = data.first_or_default(RS2_STREAM_POSE))    {
    //     rs2_pose pose_sample = pose_frame.get_pose_data();
    //     std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z << std::endl;
    //     //...
    // }

        if (rs2::motion_frame gyro_frame    = data.first_or_default(RS2_STREAM_GYRO))   algo.process(gyro_frame);
        if (rs2::motion_frame accel_frame   = data.first_or_default(RS2_STREAM_ACCEL))  algo.process(accel_frame);
        
        float3 theta3 = algo.get_theta();
        // std::cout << "rot: " << theta3.x << " " << theta3.y << " " << theta3.z <<std::endl;
        float3 theta3_degrees;
        theta3_degrees.x = theta3.x * 180 / PI;
        theta3_degrees.y = theta3.y * 180 / PI;
        theta3_degrees.z = theta3.z * 180 / PI;
        std::cout << "deg: " << theta3_degrees.x << " " << theta3_degrees.y << " " << theta3_degrees.z <<std::endl;

        // float3 theta3_degrees;
        // theta3_degrees.x = theta3.x * 180 / PI;
        // theta3_degrees.y = (theta3.y-PI) * 180 / PI;
        // theta3_degrees.z = (theta3.z + PI / 2) * 180 / PI;
        // std::cout << "deg: " << theta3_degrees.x << " " << theta3_degrees.y << " " << theta3_degrees.z <<std::endl;
        
        // theta3.x = theta3_degrees.x * PI / 180.0;
        // theta3.y = theta3_degrees.y * PI / 180.0;
        // theta3.z = theta3_degrees.z * PI / 180.0;
        

        Eigen::Matrix3f m = algo.get_rotation_matrix(theta3);
         std::cout << "Rotation: " << std::endl << m << std::endl;
        
        data = data.apply_filter(align);
        // data = data.apply_filter(dec);

        // To make sure far-away objects are filtered proportionally
        // we try to switch to disparity domain
        // data = data.apply_filter(depth2disparity);

        // // Apply spatial filtering
        // data = data.apply_filter(spat);

        // // // Apply temporal filtering
        // data = data.apply_filter(temp);

        // // // If we are in disparity domain, switch back to depth
        // data = data.apply_filter(disparity2depth);

        //// Apply color map for visualization of depth
        // data = data.apply_filter(color_map);
        
        rs2::video_frame color_frame = data.get_color_frame();//

        float width  = color_frame.get_width();//<rs2::video_frame>().get_width();
        float height = color_frame.get_height();
        // std::cout << "Got Color frame" << " " << width << " " << height << std::endl;

        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
        cv::cvtColor  (image, image, cv::COLOR_BGR2RGB);
        // char rname[256]; sprintf(rname, "rgb/frame%.3d.png", frame_num);
        // std::cout << rname << std::endl;
        // cv::imwrite(rname, image);        
        //rs2::frame rdepth = color_map(data.get_depth_frame()); // Find and colorize the depth data
        rs2::video_frame depth_frame = data.get_depth_frame();//.apply_filter(color_map);
        float dwidth  = depth_frame.get_width();//<rs2::video_frame>().get_width();
        float dheight = depth_frame.get_height();        
        // std::cout << "Got Depth frame" << " " << dwidth << " " << dheight << std::endl;
        cv::Mat dimage(cv::Size(dwidth, dheight), CV_16UC1, (void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);

        // cv::Mat image = asus.getRGB();
        // cv::Mat dimage = asus.getDepthmap(7);

        cv::imshow("rgb", image);
        // cv::imshow("d", dimage);
        cv::waitKey(1);
        // char dname[256]; sprintf(dname, "depth/frame%.3d.png", frame_num);
        // cv::imwrite(dname, dimage);

        // pc.map_to(color_frame);
        // points = pc.calculate(depth_frame);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr scloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // scloud = points_to_pclxyz(points);
        // scloud = PCL_Conversion(points, color_frame);
        // viewer.removeAllPointClouds();
        // viewer.addPointCloud<pcl::PointXYZRGB >(scloud, "11");
        // viewer.spinOnce();
        
        // // saveFrameTUMFormat(image, dimage, frame_num);      
        // saveFrameTUMFormatIMU(image, dimage, m, frame_num);
        frame_num++;
        // std::cout << "Frame: " << frame_num++ << std::endl;
        }

    }
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
// 	LOG(INFO) << "Starting test";
    
    return 0;
}
