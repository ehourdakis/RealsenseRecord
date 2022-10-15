#ifndef REALSENSE_RECORD_ROS_PUBLISHER
#define REALSENSE_RECORD_ROS_PUBLISHER

#include <iostream>
#include <fstream>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <termios.h>
#include <mutex>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <experimental/filesystem> 

#include <realsense_record_publisher/index_reader.h>
#include <Eigen/StdVector>

namespace fs = std::experimental::filesystem;

// load an eigen matrix from a text file
template<typename MatrixType, typename valuetype>
bool load_matrix_from_file (const std::string & path, const char accel, MatrixType &out) {
    std::ifstream indata;
    indata.open(path);

    if(!indata.is_open()) {
        std::cerr << "[load_matrix_from_file] " << "Could not open file " << path << std::endl; 
        return false;
    };

    std::string line;
    std::vector<valuetype> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, accel)){//',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    out = Eigen::Map<const Eigen::Matrix<typename MatrixType::Scalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
    return true;

}

namespace realsense_record_ros_publisher 
{

    struct CameraCalibrationEntry
    {
        double fx, fy, cx, cy;
        double k1, k2, k3;
        double p1, p2;
    };

    class RealsenseRecordROSPublisher {
    public:
        RealsenseRecordROSPublisher(
            const ros::NodeHandle& nh, 
            const ros::NodeHandle& nhp, 
            const std::string& rgb_info_topic_name = "/camera/rgb/camera_info",
		    const std::string& rgb_image_topic_name = "/camera/rgb/image_raw", 
		    const std::string& depth_info_topic_name = "/camera/depth_registered/camera_info",
		    const std::string& depth_image_topic_name = "/camera/depth_registered/image_raw");
    
        ~RealsenseRecordROSPublisher();
    private:
        unsigned int _fps = 30;
        bool _paused = true;
        std::string _dataset_directory;
        std::string _rgb_index_file;
        std::string _rgb_calibration_filename;
        std::string _rgb_distortion_coefficients_filename;
        std::string _depth_index_file;
        
        std::string _rgb_image_topic_name;
        std::string _rgb_info_topic_name;
        std::string _depth_image_topic_name;
        std::string _depth_info_topic_name;

        std::shared_ptr<ros::Publisher> _prgb_info_pub_;
        std::shared_ptr<ros::Publisher> _pdepth_info_pub_;
        std::shared_ptr<image_transport::Publisher> _prgb_image_pub_;
        std::shared_ptr<image_transport::Publisher> _pdepth_image_pub_;
        
        std::shared_ptr<IndexReader> _index_rgb;
        std::shared_ptr<IndexReader> _index_dep;

        // The image_transport based image publishers and the image_transport object
	    std::shared_ptr<image_transport::ImageTransport> _pimage_transport;

        // The main loop thread
        std::unique_ptr<std::thread> _pmain_loop_thread;

        // The calibrations retrieved by the realsense camera
        std::shared_ptr<CameraCalibrationEntry> _rgb_calibration;
        
        // Simulation time
        ros::Time _simulation_time;

        // ROS public node handle
        ros::NodeHandle nh_;  
        // ROS private node handle
        ros::NodeHandle nhp_;  

        // The main loop function
        void MainLoop();

        // Load calibration from file
        bool LoadCalibration();

        // Initialize the index file parsers
        bool InitializeIndexReaders();

        sensor_msgs::ImagePtr CreateDepthImageMsg(
            const cv::Mat_<uchar>& image, 
            const ros::Time& stamp);

        sensor_msgs::ImagePtr CreateRGBImageMsg(
            const cv::Mat& image, 
            const ros::Time& stamp);
    };

    // Non-blocking getch
    int kbhit();  
}

#endif