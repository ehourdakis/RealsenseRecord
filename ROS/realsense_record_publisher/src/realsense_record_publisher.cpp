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
        nh_(nh), 
		nhp_(nhp),
		_rgb_image_topic_name(rgb_image_topic_name),
		_rgb_info_topic_name(rgb_info_topic_name),
		_depth_image_topic_name(depth_image_topic_name),
		_depth_info_topic_name(depth_info_topic_name),
		_pmain_loop_thread(nullptr),
		_simulation_time(ros::Time::now())                                          
    {
        if (!nhp_.getParam("dataset_directory", _dataset_directory))
		{
			ROS_ERROR("Dataset directory not set.\n");
			ros::shutdown();
			return;
		}
        if (!nhp_.getParam("rgb_index_file", _rgb_index_file))
		{
			ROS_ERROR("RGB file index not set.\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("rgb_calibration_filename", _rgb_calibration_filename))
		{
			ROS_ERROR("RGB camera calibration not set.\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("rgb_distortion_coefficients_filename", _rgb_distortion_coefficients_filename))
		{
			ROS_ERROR("RGB distortion ceofficients not found.\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("depth_index_file", _depth_index_file))
		{
			ROS_ERROR("Depth file index not set.\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory)) {
			ROS_ERROR("Directory does not exist");
			ros::shutdown();
			return;
		}
		// Verify that files exist
		if (!fs::exists(_dataset_directory + _rgb_index_file))
		{
			ROS_ERROR("RGB index file does not exist. Have you syncrhonized the data?\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory + _rgb_calibration_filename))
		{
			ROS_ERROR("RGB calibration file does not exist!\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(_dataset_directory + _depth_index_file))
		{
			ROS_ERROR("Depth index file does not exist! . Have you syncrhonized the data?\n");
			ros::shutdown();
			return;
		}

		// Retrieve RealsenseRecord stored calibration from file
		if(!LoadCalibration())
		{
			ROS_ERROR("Cound not load calibration.\n");
			ros::shutdown();
			return;
		}

		if(!InitializeIndexReaders())
		{
			ROS_ERROR("Count not initialize the index file readers.\n");
			ros::shutdown();
			return;
		}
		
		// Initialize CameraInfo publishers
  		_prgb_info_pub_ = std::make_shared<ros::Publisher>( 
			nh_.advertise<sensor_msgs::CameraInfo>(_rgb_info_topic_name, 5) );
		
		_pdepth_info_pub_ = std::make_shared<ros::Publisher>( 
			nh_.advertise<sensor_msgs::CameraInfo>(_depth_info_topic_name, 5) );

		// Initialize the image transport object
		_pimage_transport.reset( new image_transport::ImageTransport(nh) );
		
		// Image publishers
		_prgb_image_pub_ = std::make_shared<image_transport::Publisher>( 
			_pimage_transport->advertise(_rgb_image_topic_name, 1) );
		
		_pdepth_image_pub_ = std::make_shared<image_transport::Publisher>( 
			_pimage_transport->advertise(_depth_image_topic_name, 1) );

		// Start the main loop
		 _pmain_loop_thread = std::make_unique<std::thread>(&RealsenseRecordROSPublisher::MainLoop, this);
    }


    RealsenseRecordROSPublisher::~RealsenseRecordROSPublisher() 
    {
    	if (_pmain_loop_thread)
		{
			if (_pmain_loop_thread->joinable())
       		{
            	_pmain_loop_thread->join();
        	}
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
		bool stopped = false;

		bool bdata = true; // true if non of the IndexReaders is eof
		bdata &= _index_rgb->load_data();
		bdata &= _index_dep->load_data();

		while (ros::ok() && bdata)
		{
			cv::Mat rgb_frame = cv::imread(_index_rgb->get_current_filename(),-1);
			cv::Mat depth_frame = cv::imread(_index_dep->get_current_filename(),-1);

			// cv::imshow("test", depth_frame);
			// cv::waitKey(1);
		
			// Create rgb camera info messages
			sensor_msgs::CameraInfo rgb_info;
			rgb_info.header.seq = seq_id;
			rgb_info.header.stamp = _simulation_time;
			rgb_info.header.frame_id = "camera_color_optical_frame";
			rgb_info.height = rgb_frame.rows;
			rgb_info.width = rgb_frame.cols;
			rgb_info.distortion_model = "plumb_bob";
			rgb_info.D = {_rgb_calibration->k1, _rgb_calibration->k2, 
						  _rgb_calibration->k3, _rgb_calibration->p1, _rgb_calibration->p2};
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
		_rgb_calibration = std::make_shared<CameraCalibrationEntry>();
		
		// load the rgb intrinsics data
		Eigen::Matrix3d intrinsics_eig; 
		bool loaded = load_matrix_from_file<Eigen::Matrix3d, double> (_dataset_directory + std::string("/rgb.intrinsics"), ',', intrinsics_eig);
		if(!loaded) {
			ROS_WARN_STREAM("Count not load " << _dataset_directory + std::string("/rgb.intrinsics") << " file");
			return false;
		}

		_rgb_calibration->fx = intrinsics_eig(0,0);
		_rgb_calibration->fy = intrinsics_eig(1,1);
		_rgb_calibration->cx = intrinsics_eig(0,2);
		_rgb_calibration->cy = intrinsics_eig(1,2);

		ROS_INFO_STREAM("Intrinsics. fx: " << _rgb_calibration->fx << " fy: " << _rgb_calibration->fy << " cx: " << _rgb_calibration->cx << " cy: " << _rgb_calibration->cy);

		// load the rgb distortion
		Eigen::Matrix<float, 1, 5> dist_coeffs_eig;
		loaded = load_matrix_from_file<Eigen::Matrix<float, 1, 5>, float> (_dataset_directory + std::string("/rgb.distortion"), ' ', dist_coeffs_eig);
		if(!loaded) {
			ROS_WARN_STREAM("Count not load " << _dataset_directory + std::string("/rgb.distortion") << " file");
			return false;
		}
		
		_rgb_calibration->p1 = dist_coeffs_eig(0,0);
		_rgb_calibration->p2 = dist_coeffs_eig(0,1);
		_rgb_calibration->k1 = dist_coeffs_eig(0,2);
		_rgb_calibration->k2 = dist_coeffs_eig(0,3);
		_rgb_calibration->k3 = dist_coeffs_eig(0,4);

		ROS_INFO_STREAM("Distortion. p1: " << _rgb_calibration->p1 << " p2: " << _rgb_calibration->p2 << " k1: " << _rgb_calibration->k1 << " k2: " << _rgb_calibration->k2 << " k3: " << _rgb_calibration->k3);

		return true;
	}

	bool RealsenseRecordROSPublisher::InitializeIndexReaders() {
		// initialize the IndexReaders
		_index_rgb.reset(new IndexReader());
		_index_dep.reset(new IndexReader());

		if(!_index_rgb->load_index(_dataset_directory + std::string("/rgb_aligned.txt"))) {
			ROS_WARN("Could not load rgb index file.");
			return false;
		}
		if(!_index_dep->load_index(_dataset_directory + std::string("/depth_aligned.txt"))) {
			ROS_WARN("Could not load depth index file");
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
