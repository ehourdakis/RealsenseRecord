#include "realsense_record_publisher/realsense_record_publisher.h"

#include <stdio.h>
#include <sys/ioctl.h> // For FIONREAD
#include <termios.h>
#include <stdbool.h>

namespace realsense_record_ros_publisher 
{
    RealsenseRecordROSPublisher::RealsenseRecordROSPublisher(
        const ros::NodeHandle& nh, 
        const ros::NodeHandle& nhp,
        const std::string& rgb_info_topic_name,
        const std::string& rgb_image_topic_name,
        const std::string& depth_info_topic_name,
        const std::string& depth_image_topic_name):
        _rgb_image_topic_name(rgb_image_topic_name),
        _rgb_info_topic_name(rgb_info_topic_name),
        _depth_image_topic_name(depth_image_topic_name),
        _depth_info_topic_name(depth_info_topic_name),
        _pmain_loop_thread(nullptr),
        _simulation_time(ros::Time::now()),
        nh_(nh),
        nhp_(nhp)
    {
		std::string dataset_directory; 
		if (!nhp_.getParam("dataset_directory", dataset_directory))
		{
			ROS_ERROR("Dataset directory not set.\n");
			ros::shutdown();
			return;
		} else _dataset_directory = dataset_directory;

		std::string rgb_index_file; 
		if (!nhp_.getParam("rgb_index_file", rgb_index_file))
		{
			ROS_ERROR("RGB file index not set.\n");
			ros::shutdown();
			return;
		} else _rgb_index_file = rgb_index_file;

		std::string rgb_calibration_filename; 
		if (!nhp_.getParam("rgb_calibration_filename", rgb_calibration_filename))
		{
			ROS_ERROR("RGB camera calibration not set.\n");
			ros::shutdown();
			return;
		} else _rgb_calibration_filename = rgb_calibration_filename;

		std::string rgb_distortion_coefficients_filename; 
		if (!nhp_.getParam("rgb_distortion_coefficients_filename", rgb_distortion_coefficients_filename))
		{
			ROS_ERROR("RGB distortion coefficients not set.\n");
			ros::shutdown();
			return;
		} else _rgb_distortion_coefficients_filename = rgb_distortion_coefficients_filename;

		std::string depth_index_file; 
		if (!nhp_.getParam("depth_index_file", depth_index_file))
		{
			ROS_ERROR("Depth file index not set.\n");
			ros::shutdown();
			return;
		} else _depth_index_file = depth_index_file;

		if (!nhp_.getParam("rgb_info_topic_name", _rgb_info_topic_name))
		{
			ROS_ERROR("RGB info topic name not set.\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("rgb_image_topic_name", _rgb_image_topic_name))
		{
			ROS_ERROR("RGB image topic name not set.\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("depth_info_topic_name", _depth_info_topic_name))
		{
			ROS_ERROR("depth info topic name not set.\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("depth_image_topic_name", _depth_image_topic_name))
		{
			ROS_ERROR("depth image topic name not set.\n");
			ros::shutdown();
			return;
		}

		if (!fs::exists(_dataset_directory)) {
			ROS_ERROR("Directory does not exist");
			ros::shutdown();
			return;
		} else
			ROS_INFO_STREAM("Dataset directory set to: " << _dataset_directory);
			
		// Verify that files exist
		if (!fs::exists(_dataset_directory / _rgb_index_file))
		{
			ROS_ERROR("RGB index file does not exist! Have you synchronized the data?\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory / _rgb_calibration_filename))
		{
			ROS_ERROR("RGB calibration file does not exist!\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory / _rgb_distortion_coefficients_filename))
		{
			ROS_ERROR("RGB distortion coefficients file does not exist!\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory / _depth_index_file))
		{
			ROS_ERROR("Depth index file does not exist! Have you synchronized the data?\n");
			ros::shutdown();
			return;
		}

		_rgb_calibration = std::make_unique<CameraCalibrationEntry>();

		// Retrieve RealsenseRecord stored calibration from file
		if(!LoadCalibration())
		{
			ROS_ERROR("Could not load calibration.\n");
			ros::shutdown();
			return;
		}

		if(!InitializeIndexReaders())
		{
			ROS_ERROR("Could not initialize the index file readers.\n");
			ros::shutdown();
			return;
		}

		ROS_INFO_STREAM("Publishing to RGB img + info topics " << _rgb_image_topic_name << " + " << _rgb_info_topic_name);
		ROS_INFO_STREAM("Publishing to depth img + info topics " << _depth_image_topic_name << " + " << _depth_info_topic_name);

		//// ROS-related initialization

		// Initialize CameraInfo publishers
  		_prgb_info_pub_ = std::make_unique<ros::Publisher>( 
			nh_.advertise<sensor_msgs::CameraInfo>(_rgb_info_topic_name, 5) );
		
		_pdepth_info_pub_ = std::make_unique<ros::Publisher>( 
			nh_.advertise<sensor_msgs::CameraInfo>(_depth_info_topic_name, 5) );

		// Initialize the image transport object
		_pimage_transport.reset( new image_transport::ImageTransport(nh) );
		
		// Image publishers
		_prgb_image_pub_ = std::make_unique<image_transport::Publisher>( 
			_pimage_transport->advertise(_rgb_image_topic_name, 1) );
		
		_pdepth_image_pub_ = std::make_unique<image_transport::Publisher>( 
			_pimage_transport->advertise(_depth_image_topic_name, 1) );

		// Start the main loop
		 _pmain_loop_thread = std::make_unique<std::thread>(&RealsenseRecordROSPublisher::MainLoop, this);
    }

    RealsenseRecordROSPublisher::~RealsenseRecordROSPublisher() 
    {
	if (_pmain_loop_thread && _pmain_loop_thread->joinable()) {
            	_pmain_loop_thread->join();
	}
    }

    sensor_msgs::ImagePtr RealsenseRecordROSPublisher::CreateDepthImageMsg(
		const cv::Mat& opencv_image, 
		const ros::Time& stamp)
	{
		std_msgs::Header header;
		header.stamp = stamp;
		sensor_msgs::ImagePtr pimage_msg = cv_bridge::CvImage(
			header, 
			"mono16", 
			opencv_image).toImageMsg();
		
		return pimage_msg;
	}

    sensor_msgs::ImagePtr RealsenseRecordROSPublisher::CreateRGBImageMsg(
		const cv::Mat& opencv_image, 
		const ros::Time& stamp)
	{
		std_msgs::Header header;
		header.stamp = stamp;
		sensor_msgs::ImagePtr pimage_msg = cv_bridge::CvImage(
			header, 
			"bgr8", 
			opencv_image).toImageMsg();
		
		return pimage_msg;
	}

	void RealsenseRecordROSPublisher::MainLoop() 
	{
		ros::Rate rate(_fps);
		// Start simulation time
		_simulation_time = ros::Time(0);
		
		uint32_t seq_id = 0;

		bool bdata = true; // true if none of the IndexReaders is eof
		bdata &= _index_rgb->load_data();
		bdata &= _index_dep->load_data();

		while (ros::ok() && bdata)
		{
			cv::Mat rgb_frame = cv::imread(_index_rgb->get_current_filename(), -1); // cv::IMREAD_UNCHANGED
			cv::Mat depth_frame = cv::imread(_index_dep->get_current_filename(), -1);
		
			//_simulation_time = ros::Time::now();

			// Create rgb camera info messages
			sensor_msgs::CameraInfo rgb_info;
			rgb_info.header.seq = seq_id;
			rgb_info.header.stamp = _simulation_time;
			rgb_info.header.frame_id = "camera_color_optical_frame";
			rgb_info.height = rgb_frame.rows;
			rgb_info.width = rgb_frame.cols;
			rgb_info.distortion_model = "plumb_bob";
			rgb_info.D = {_rgb_calibration->k1, _rgb_calibration->k2, 
						  _rgb_calibration->p1, _rgb_calibration->p2, _rgb_calibration->k3};
			rgb_info.K = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 1.0};
			rgb_info.R = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};			
			rgb_info.P = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, 0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 0, 1.0, 0};
			rgb_info.binning_x = rgb_info.binning_y = 0;
			_prgb_info_pub_->publish(rgb_info);

			// Create depth camera info messages
			sensor_msgs::CameraInfo depth_info;
			depth_info.header.seq = seq_id;
			depth_info.header.stamp = _simulation_time;
			depth_info.header.frame_id = "camera_depth_optical_frame";
			depth_info.height = depth_frame.rows;
			depth_info.width = depth_frame.cols;
			depth_info.distortion_model = "plumb_bob";
			depth_info.D = {0, 0, 0, 0, 0};
			depth_info.K = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 1.0};
			depth_info.R = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};			
			depth_info.P = {_rgb_calibration->fx, 0, _rgb_calibration->cx,
							0, 0, _rgb_calibration->fy, _rgb_calibration->cy,
							0, 0, 0, 1.0, 0};
			depth_info.binning_x = depth_info.binning_y = 0;
			_pdepth_info_pub_->publish(depth_info);

			// Publish images
			sensor_msgs::ImagePtr rgb_frame_msg = CreateRGBImageMsg(rgb_frame, _simulation_time);
			_prgb_image_pub_->publish(rgb_frame_msg);
			sensor_msgs::ImagePtr depth_frame_msg = CreateDepthImageMsg(depth_frame, _simulation_time);
			_pdepth_image_pub_->publish(depth_frame_msg);

			ROS_INFO_STREAM("Frame " << seq_id);
			rate.sleep();
			_simulation_time += ros::Duration(1.0 / _fps);
			seq_id++;

			fflush(stdin);
    		
			if (kbhit()) {
				int ch = getchar();
				if (ch == 32) _paused = !_paused;
				ROS_INFO_STREAM((_paused?"":"Not ") << "Paused...\n");
			}

			if(_paused)
			{
				fflush(stdin);
				if(getchar()==32) _paused = false;
			}

			bdata &= _index_rgb->load_data();
			bdata &= _index_dep->load_data();
		}

		ROS_INFO("Published all images from dataset.");
	}

	bool RealsenseRecordROSPublisher::LoadCalibration()
	{	
		// load the rgb intrinsics data
		Eigen::Matrix3d intrinsics_eig; 
		bool loaded = load_matrix_from_file<Eigen::Matrix3d, double> (_dataset_directory / _rgb_calibration_filename, ',', intrinsics_eig);
		if(!loaded) {
			ROS_WARN_STREAM("Could not load " << _dataset_directory / _rgb_calibration_filename << " file");
			return false;
		}

		_rgb_calibration->fx = intrinsics_eig(0,0);
		_rgb_calibration->fy = intrinsics_eig(1,1);
		_rgb_calibration->cx = intrinsics_eig(0,2);
		_rgb_calibration->cy = intrinsics_eig(1,2);

		ROS_INFO_STREAM("Intrinsics. fx: " << _rgb_calibration->fx << " fy: " << _rgb_calibration->fy << " cx: " << _rgb_calibration->cx << " cy: " << _rgb_calibration->cy);

		// load the rgb distortion
		Eigen::Matrix<float, 1, 5> dist_coeffs_eig;
		loaded = load_matrix_from_file<Eigen::Matrix<float, 1, 5>, float> (_dataset_directory / _rgb_distortion_coefficients_filename, ' ', dist_coeffs_eig);
		if(!loaded) {
			ROS_WARN_STREAM("Could not load " << _dataset_directory / _rgb_distortion_coefficients_filename);
			return false;
		}
		
		_rgb_calibration->k1 = dist_coeffs_eig(0,0);
		_rgb_calibration->k2 = dist_coeffs_eig(0,1);
		_rgb_calibration->p1 = dist_coeffs_eig(0,2);
		_rgb_calibration->p2 = dist_coeffs_eig(0,3);
		_rgb_calibration->k3 = dist_coeffs_eig(0,4);

		ROS_INFO_STREAM("Distortion. p1: " << _rgb_calibration->p1 << " p2: " << _rgb_calibration->p2 << " k1: " << _rgb_calibration->k1 << " k2: " << _rgb_calibration->k2 << " k3: " << _rgb_calibration->k3);

		return true;
	}

	bool RealsenseRecordROSPublisher::InitializeIndexReaders() {
		// initialize the IndexReaders for depth and rgb image sets.
		_index_rgb.reset(new IndexReader());
		_index_dep.reset(new IndexReader());

		if(!_index_rgb->load_index(_dataset_directory / _rgb_index_file )) {
			ROS_WARN_STREAM("Could not load rgb index file." << _dataset_directory / _rgb_index_file);
			return false;
		}
		if(!_index_dep->load_index(_dataset_directory / _depth_index_file)) {
			ROS_WARN_STREAM("Could not load depth index file" << _dataset_directory / _depth_index_file);
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
  ros::init(argc, argv, "realsense_record_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  realsense_record_ros_publisher::RealsenseRecordROSPublisher realsense_record_publisher(nh, nhp);
  
  ROS_INFO("Press spacebar to start or pause the publisher. When paused press any key to publish one frame.");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}
