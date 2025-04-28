#ifndef REALSENSE_RECORD_ROS_PUBLISHER
#define REALSENSE_RECORD_ROS_PUBLISHER

#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <string>
#include <mutex>
#include <thread>
#include <memory>
#include <filesystem> 

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <realsense_record_publisher/index_reader.h>
#include <Eigen/StdVector>

namespace fs = std::filesystem;

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

    class RealsenseRecordROSPublisher
    {
    public:
        RealsenseRecordROSPublisher(
            const rclcpp::Node::SharedPtr node,
            const std::string& rgb_info_topic_name = "/camera/color/camera_info",
            const std::string& rgb_image_topic_name = "/camera/color/image_raw",
            const std::string& depth_info_topic_name = "/camera/aligned_depth_to_color/camera_info",
            const std::string& depth_image_topic_name = "/camera/aligned_depth_to_color/image_raw");
    
        ~RealsenseRecordROSPublisher();
        void init(); 

    private:
        unsigned int _fps = 30;
        bool _paused = true;
        fs::path _dataset_directory;
        fs::path _rgb_index_file;
        fs::path _rgb_calibration_filename;
        fs::path _rgb_distortion_coefficients_filename;
        fs::path _depth_index_file;
        
        std::string _rgb_image_topic_name;
        std::string _rgb_info_topic_name;
        std::string _depth_image_topic_name;
        std::string _depth_info_topic_name;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _prgb_info_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _pdepth_info_pub_;

        // The image_transport based image publishers
        std::unique_ptr<image_transport::Publisher> _prgb_image_pub_;
        std::unique_ptr<image_transport::Publisher> _pdepth_image_pub_;
        
        std::unique_ptr<IndexReader> _index_rgb;
        std::unique_ptr<IndexReader> _index_dep;

        // The main loop thread
        std::unique_ptr<std::thread> _pmain_loop_thread;

        // The calibration for the rgb camera
        std::unique_ptr<CameraCalibrationEntry> _rgb_calibration;
        
        // Simulation time
        rclcpp::Time _simulation_time;

        // ROS public node handle
        rclcpp::Node::SharedPtr _node;

        // The main loop function
        void MainLoop();

        // Load calibration from file
        bool LoadCalibration();

        // Initialize the index file parsers
        bool InitializeIndexReaders();

        // Create a depth ros image (mono16)
        sensor_msgs::msg::Image::SharedPtr CreateDepthImageMsg(
            const cv::Mat& image,
            const rclcpp::Time& stamp);

        // Create a color ros image (bgr8)
        sensor_msgs::msg::Image::SharedPtr CreateRGBImageMsg(
            const cv::Mat& image, 
            const rclcpp::Time& stamp);
    };

    // Non-blocking getch
    int kbhit();  
}

#endif
