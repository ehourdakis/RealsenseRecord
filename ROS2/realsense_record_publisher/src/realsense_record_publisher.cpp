#include "realsense_record_publisher/realsense_record_publisher.h"

#include <stdio.h>
#include <sys/ioctl.h> // For FIONREAD
#include <termios.h>
#include <stdbool.h>

namespace realsense_record_ros_publisher 
{
    RealsenseRecordROSPublisher::RealsenseRecordROSPublisher(
        const rclcpp::Node::SharedPtr node,
        const std::string& rgb_info_topic_name,
        const std::string& rgb_image_topic_name, 
        const std::string& depth_info_topic_name,
        const std::string& depth_image_topic_name) :
        _rgb_image_topic_name(rgb_image_topic_name),
        _rgb_info_topic_name(rgb_info_topic_name),
        _depth_image_topic_name(depth_image_topic_name),
        _depth_info_topic_name(depth_info_topic_name),
        _pmain_loop_thread(nullptr),
        _simulation_time(rclcpp::Clock().now()),                                          
        _node(node)
    {
		// Declare parameters
		node->declare_parameter<std::string>("dataset_directory", "");
		node->declare_parameter<std::string>("rgb_index_file", "");
		node->declare_parameter<std::string>("rgb_calibration_filename", "");
		node->declare_parameter<std::string>("rgb_distortion_coefficients_filename", "");
		node->declare_parameter<std::string>("depth_index_file", "");
		node->declare_parameter<std::string>("rgb_info_topic_name", "rgb/camera_info");
		node->declare_parameter<std::string>("rgb_image_topic_name", "rgb/image_raw");
		node->declare_parameter<std::string>("depth_info_topic_name", "depth/camera_info");
		node->declare_parameter<std::string>("depth_image_topic_name", "depth/image_raw");
		node->declare_parameter<double>("fps", 30.0);

		// Get parameter values
		_dataset_directory = node->get_parameter("dataset_directory").as_string();
		_rgb_index_file = node->get_parameter("rgb_index_file").as_string();
		_rgb_calibration_filename = node->get_parameter("rgb_calibration_filename").as_string();
		_rgb_distortion_coefficients_filename = node->get_parameter("rgb_distortion_coefficients_filename").as_string();
		_depth_index_file = node->get_parameter("depth_index_file").as_string();
		_rgb_info_topic_name = node->get_parameter("rgb_info_topic_name").as_string();
		_rgb_image_topic_name = node->get_parameter("rgb_image_topic_name").as_string();
		_depth_info_topic_name = node->get_parameter("depth_info_topic_name").as_string();
		_depth_image_topic_name = node->get_parameter("depth_image_topic_name").as_string();
		_fps = node->get_parameter("fps").as_double();

		// Check parameter values
		auto logger = node->get_logger();
		if (_dataset_directory.empty())
		{
			RCLCPP_ERROR(logger, "Dataset directory not set");
			rclcpp::shutdown();
			return;
		} else if (!fs::exists(_dataset_directory))
		{
			RCLCPP_ERROR(logger, "Dataset directory does not exist: %s", _dataset_directory.c_str());
			rclcpp::shutdown();
			return;
		} else
			RCLCPP_INFO_STREAM(logger, "Dataset directory set to: " << _dataset_directory);

		if (_rgb_index_file.empty())
		{
			RCLCPP_ERROR(logger, "RGB file index not set.\n");
			rclcpp::shutdown();
			return;
		}
		
		if (_rgb_calibration_filename.empty())
		{
			RCLCPP_ERROR(logger, "RGB camera calibration not set.\n");
			rclcpp::shutdown();
			return;
		}

		if (_rgb_distortion_coefficients_filename.empty())
		{
			RCLCPP_ERROR(logger, "RGB distortion coefficients not set.\n");
			rclcpp::shutdown();
			return;
		}

		if (_depth_index_file.empty())
		{
			RCLCPP_ERROR(logger, "Depth file index not set.\n");
			rclcpp::shutdown();
			return;
		}

		if (_rgb_info_topic_name.empty())
		{
			RCLCPP_ERROR(logger, "RGB info topic name not set.\n");
			rclcpp::shutdown();
			return;
		}
		if (_rgb_image_topic_name.empty())
		{
			RCLCPP_ERROR(logger, "RGB image topic name not set.\n");
			rclcpp::shutdown();
			return;
		}
		if (_depth_info_topic_name.empty())
		{
			RCLCPP_ERROR(logger, "Depth info topic name not set.\n");
			rclcpp::shutdown();
			return;
		}
		if (_depth_image_topic_name.empty())
		{
			RCLCPP_ERROR(logger, "Depth image topic name not set.\n");
			rclcpp::shutdown();
			return;
		}

		if (_fps <= 0) _fps = 30.0;

		// Verify that files exist
		if (!fs::exists(_dataset_directory / _rgb_index_file))
		{
			RCLCPP_ERROR(logger, "RGB index file does not exist! Have you synchronized the data?\n");
			rclcpp::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory / _rgb_calibration_filename))
		{
			RCLCPP_ERROR(logger, "RGB calibration file does not exist!\n");
			rclcpp::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory / _rgb_distortion_coefficients_filename))
		{
			RCLCPP_ERROR(logger, "RGB distortion coefficients file does not exist!\n");
			rclcpp::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory / _depth_index_file))
		{
			RCLCPP_ERROR(logger, "Depth index file does not exist! Have you synchronized the data?\n");
			rclcpp::shutdown();
			return;
		}

		_rgb_calibration = std::make_unique<CameraCalibrationEntry>();

		// Retrieve RealsenseRecord stored calibration from file
		if(!LoadCalibration())
		{
			RCLCPP_ERROR(logger, "Could not load calibration.\n");
			rclcpp::shutdown();
			return;
		}

		if(!InitializeIndexReaders())
		{
			RCLCPP_ERROR(logger, "Could not initialize the index file readers.\n");
			rclcpp::shutdown();
			return;
		}
    }

    // ROS-related initialization
    void RealsenseRecordROSPublisher::init()
    {
		// Initialize CameraInfo publishers
  		_prgb_info_pub_ = _node->create_publisher<sensor_msgs::msg::CameraInfo>(_rgb_info_topic_name, 5);
		_pdepth_info_pub_ = _node->create_publisher<sensor_msgs::msg::CameraInfo>(_depth_info_topic_name, 5);

		// Image publishers
		auto qos = rclcpp::SensorDataQoS().keep_last(1);
		_prgb_image_pub_ = std::make_unique<image_transport::Publisher>(image_transport::create_publisher(_node.get(), _rgb_image_topic_name, qos.get_rmw_qos_profile()));
		_pdepth_image_pub_ = std::make_unique<image_transport::Publisher>(image_transport::create_publisher(_node.get(), _depth_image_topic_name, qos.get_rmw_qos_profile()));

		// Start the main loop
		_pmain_loop_thread = std::make_unique<std::thread>(&RealsenseRecordROSPublisher::MainLoop, this);
    }

    RealsenseRecordROSPublisher::~RealsenseRecordROSPublisher() 
    {
    	if (_pmain_loop_thread && _pmain_loop_thread->joinable()) {
            	_pmain_loop_thread->join();
    	}
    }

    sensor_msgs::msg::Image::SharedPtr RealsenseRecordROSPublisher::CreateDepthImageMsg(
		const cv::Mat& opencv_image, 
		const rclcpp::Time& stamp)
	{
		std_msgs::msg::Header header;
		header.stamp = stamp;
		auto pimage_msg = cv_bridge::CvImage(
			header, 
			"mono16", 
			opencv_image).toImageMsg();
		
		return pimage_msg;
	}

    sensor_msgs::msg::Image::SharedPtr RealsenseRecordROSPublisher::CreateRGBImageMsg(
		const cv::Mat& opencv_image, 
		const rclcpp::Time& stamp)
	{
		std_msgs::msg::Header header;
		header.stamp = stamp;
		auto pimage_msg = cv_bridge::CvImage(
			header, 
			"bgr8", 
			opencv_image).toImageMsg();
		
		return pimage_msg;
	}

	void RealsenseRecordROSPublisher::MainLoop() 
	{
		rclcpp::Rate rate(static_cast<int>(_fps));
		// Start simulation time
		_simulation_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
		
		uint32_t seq_id = 0;

		bool bdata = true; // true if none of the IndexReaders is eof
		bdata &= _index_rgb->load_data();
		bdata &= _index_dep->load_data();

		auto logger = _node->get_logger();

		while (rclcpp::ok() && bdata)
		{
			cv::Mat rgb_frame = cv::imread(_index_rgb->get_current_filename(), cv::IMREAD_UNCHANGED);
			cv::Mat depth_frame = cv::imread(_index_dep->get_current_filename(), cv::IMREAD_UNCHANGED);

			//_simulation_time = _node->now();
			
			// Create rgb camera info messages
			sensor_msgs::msg::CameraInfo rgb_info;
			rgb_info.header.stamp = _simulation_time;
			rgb_info.header.frame_id = "camera_color_optical_frame";
			rgb_info.height = rgb_frame.rows;
			rgb_info.width = rgb_frame.cols;
			rgb_info.distortion_model = "plumb_bob";
			rgb_info.d = {_rgb_calibration->k1, _rgb_calibration->k2, 
						  _rgb_calibration->p1, _rgb_calibration->p2, _rgb_calibration->k3};
			rgb_info.k = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 1.0};
			rgb_info.r = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};			
			rgb_info.p = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, 0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 0, 1.0, 0};
			rgb_info.binning_x = rgb_info.binning_y = 0;
			_prgb_info_pub_->publish(rgb_info);

			// Create depth camera info messages
			sensor_msgs::msg::CameraInfo depth_info;
			depth_info.header.stamp = _simulation_time;
			depth_info.header.frame_id = "camera_depth_optical_frame";
			depth_info.height = depth_frame.rows;
			depth_info.width = depth_frame.cols;
			depth_info.distortion_model = "plumb_bob";
			depth_info.d = {0, 0, 0, 0, 0};
			depth_info.k = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 1.0};
			depth_info.r = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};			
			depth_info.p = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, 0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 0, 1.0, 0};
			depth_info.binning_x = depth_info.binning_y = 0;
			_pdepth_info_pub_->publish(depth_info);

			// Publish images
			sensor_msgs::msg::Image::SharedPtr rgb_frame_msg = CreateRGBImageMsg(rgb_frame, _simulation_time);
			_prgb_image_pub_->publish(rgb_frame_msg);
			sensor_msgs::msg::Image::SharedPtr depth_frame_msg = CreateDepthImageMsg(depth_frame, _simulation_time);
			_pdepth_image_pub_->publish(depth_frame_msg);

			RCLCPP_INFO_STREAM(logger, "Frame " << seq_id);
			rate.sleep();
			_simulation_time += rclcpp::Duration::from_seconds(1.0 / _fps);
			seq_id++;
			
			fflush(stdin);
    		
			if (kbhit()) {
				int ch = getchar();
				if (ch == 32) _paused = !_paused;
				RCLCPP_INFO_STREAM(logger, (_paused?"":"Not ") << "Paused...\n");
			}

			if(_paused)
			{
				fflush(stdin);
				if(getchar()==32) _paused = false;
			}

			bdata &= _index_rgb->load_data();
			bdata &= _index_dep->load_data();
		}

		RCLCPP_INFO(logger, "Published all images from dataset.");
	}

	bool RealsenseRecordROSPublisher::LoadCalibration()
	{	
		auto logger = _node->get_logger();

		// load the rgb intrinsics data
		Eigen::Matrix3d intrinsics_eig; 
		bool loaded = load_matrix_from_file<Eigen::Matrix3d, double> (_dataset_directory / _rgb_calibration_filename, ',', intrinsics_eig);
		if(!loaded) {
			RCLCPP_WARN_STREAM(logger, "Could not load " << _dataset_directory / _rgb_calibration_filename << " file");
			return false;
		}

		_rgb_calibration->fx = intrinsics_eig(0,0);
		_rgb_calibration->fy = intrinsics_eig(1,1);
		_rgb_calibration->cx = intrinsics_eig(0,2);
		_rgb_calibration->cy = intrinsics_eig(1,2);

		RCLCPP_INFO_STREAM(logger, "Intrinsics. fx: " << _rgb_calibration->fx << " fy: " << _rgb_calibration->fy << " cx: " << _rgb_calibration->cx << " cy: " << _rgb_calibration->cy);

		// load the rgb distortion
		Eigen::Matrix<float, 1, 5> dist_coeffs_eig;
		loaded = load_matrix_from_file<Eigen::Matrix<float, 1, 5>, float> (_dataset_directory / _rgb_distortion_coefficients_filename, ' ', dist_coeffs_eig);
		if(!loaded) {
			RCLCPP_INFO_STREAM(logger, "Could not load " << _dataset_directory / _rgb_distortion_coefficients_filename);
			return false;
		}
		
		_rgb_calibration->k1 = dist_coeffs_eig(0,0);
		_rgb_calibration->k2 = dist_coeffs_eig(0,1);
		_rgb_calibration->p1 = dist_coeffs_eig(0,2);
		_rgb_calibration->p2 = dist_coeffs_eig(0,3);
		_rgb_calibration->k3 = dist_coeffs_eig(0,4);

		RCLCPP_INFO_STREAM(logger, "Distortion. p1: " << _rgb_calibration->p1 << " p2: " << _rgb_calibration->p2 << " k1: " << _rgb_calibration->k1 << " k2: " << _rgb_calibration->k2 << " k3: " << _rgb_calibration->k3);

		return true;
	}

	bool RealsenseRecordROSPublisher::InitializeIndexReaders()
	{	
		auto logger = _node->get_logger();

		// initialize the IndexReaders for depth and rgb image sets.
		_index_rgb = std::make_unique<IndexReader>();
		_index_dep = std::make_unique<IndexReader>();

		if(!_index_rgb->load_index(_dataset_directory / _rgb_index_file )) {
			RCLCPP_WARN_STREAM(logger, "Could not load rgb index file." << _dataset_directory / _rgb_index_file);
			return false;
		}
		if(!_index_dep->load_index(_dataset_directory / _depth_index_file)) {
			RCLCPP_WARN_STREAM(logger, "Could not load depth index file" << _dataset_directory / _depth_index_file);
			return false;
		}

		return true;
	}
	
	/**
	 * @brief Non-blocking character reading.
	 * @ref https://stackoverflow.com/a/33201364
	 */
	int kbhit() {
		static bool initflag = false;
		static const int STDIN = 0;

		if (!initflag) {
			// Use termios to turn off line buffering
			struct termios term;
			tcgetattr(STDIN, &term);
			term.c_lflag &= ~ICANON;
			tcsetattr(STDIN, TCSANOW, &term);
			setbuf(stdin, NULL);
			initflag = true;
		}

		int nbbytes;
		ioctl(STDIN, FIONREAD, &nbbytes);  // 0 is STDIN
		return nbbytes;
	}

} // end namespace realsense_record_ros_publisher


int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.enable_rosout(true);
  //options.use_intra_process_comms(true);
  auto node = rclcpp::Node::make_shared("realsense_record_publisher", options);

  auto publisher = std::make_shared<realsense_record_ros_publisher::RealsenseRecordROSPublisher>(node);
  publisher->init();

  RCLCPP_INFO(node->get_logger(), "Press spacebar to start or pause the publisher. When paused press any key to publish one frame.");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
