// This is a recorder that stores frames as RGB Depth + multiple IMU acc,rot readings vector
// It uses the device raw callback and drops the frames with the keyframes as is with their timetamps
// Recorded frames must be post processed for synchronization

#include "helpers.h"        // Basic data structures for holding frames

#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <fstream>
#include <boost/program_options.hpp> // command line argument options
#include <boost/algorithm/string.hpp>

namespace po = boost::program_options;

// Table of acceptable frame size / fps pairs for the D455 camera.
std::map<std::pair<int, int>, std::vector<int>> fps_table = {
    {{1280, 800}, {30, 15, 10, 5}},
    {{1280, 720}, {30, 15, 10, 5}},
    {{848, 480}, {60, 30, 15, 5}},
    {{640, 480}, {60, 30, 15, 5}},
    {{640, 360}, {90, 60, 30, 15, 5}},
    {{480, 270}, {90, 60, 30, 15, 5}}
};

// validate the value of the accelerometer fps
bool validate_acc_fps(const int option_value) {
  const std::vector<int> allowed_values = {250, 63}; // allowed values
  if (std::find(allowed_values.begin(), allowed_values.end(), option_value) == allowed_values.end()) {
    std::cout << "Invalid accelerometer fps. Allowed values are {250, 63}\n";
    return false;
  }
  return true;
}

// validate the value of the gyroscope fps
bool validate_gyro_fps(const int option_value) {
  const std::vector<int> allowed_values = {400, 200}; // allowed values
  if (std::find(allowed_values.begin(), allowed_values.end(), option_value) == allowed_values.end()) {
    std::cout << "Invalid gyro fps. Allowed values are {400, 200}\n";
    return false;
  }
  return true;
}

// validate whether the combination frame size / fps exists
bool validate_frame_properties(int frame_width, int frame_height, int opt_framerate) {
    auto it = fps_table.find({frame_width, frame_height});
    if (it == fps_table.end()) {
        std::cout << "Invalid frame resolution: " << std::to_string(frame_width) << "x" << std::to_string(frame_height) << std::endl;
        std::cout << "Available resolutions are: " << std::endl;
        for (auto f : fps_table) {
            std::cout << "  " << f.first.first << "x" << f.first.second << std::endl;
        }

        return false;
    }

    int fps = opt_framerate;
    auto fps_vec = it->second;
    if (std::find(fps_vec.begin(), fps_vec.end(), fps) == fps_vec.end()) {
        std::cout << "Invalid frame rate for resolution " << frame_width << "x" << std::to_string(frame_height) << ": " << std::to_string(opt_framerate) << std::endl;
        std::cout << "Available framerates for " << frame_width << "x" << frame_height << ":" << std::endl;
        for (auto f : fps_vec) {
            std::cout << "  " << f << " Hz" << std::endl;
        }

        return false;
    }

    return true;
}

int main(int argc, char * argv[]) 
{
    // Default application parameters 
    std::string data_dir = "/home/";
    int dataset_size = 500;
    int opt_framerate = 90;
    int frame_width = 640;
    int frame_height = 360;
    int acc_framerate = 255;
    int gyro_framerate = 400;
    int rgb_exposure = 200;
    int depth_exposure = 10;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Help message")
        ("dataset_dir", po::value<std::string>(&data_dir)->required(), "Directory to save the recorded dataset")
        ("dataset_size", po::value<int>(&dataset_size)->required(), "Size of the recorded dataset")
        ("rgb_fps", po::value<int>(&opt_framerate)->required(), "Depth frame rate")
        ("frame_width", po::value<int>(&frame_width)->default_value(640), "Frame width")
        ("frame_height", po::value<int>(&frame_height)->default_value(360), "Frame height")
        ("acc_framerate", po::value<int>(&acc_framerate)->default_value(250), "Accelerometer framerate")
        ("gyro_framerate", po::value<int>(&gyro_framerate)->default_value(400), "Gyroscope framerate")
        ("rgb_exposure", po::value<int>(&rgb_exposure)->default_value(200), "RGB exposure")
        ("depth_exposure", po::value<int>(&depth_exposure)->default_value(10), "Depth exposure");

    po::positional_options_description p;
    p.add("dataset_dir", 1)
     .add("dataset_size", 1)
     .add("rgb_fps", 1)
     .add("frame_width", 1)
     .add("frame_height", 1)
     .add("acc_framerate", 1)
     .add("gyro_framerate", 1)
     .add("rgb_exposure", 1)
     .add("depth_exposure", 1);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        
        if (vm.count("help")) {
            std::cout << "RealsenseRecord. Record sensor data from a realsense camera" << std::endl << std::endl;
            std::cout << "Run as: " << std::endl;
            std::cout << argv[0] << "data/ 1000 30" << std::endl << std::endl << std::endl;

            std::cout << desc << "\n";

            std::cout << "Available camera frame size / frame rates:" << std::endl;
            for (const auto& kv : fps_table) {
                int width = kv.first.first;
                int height = kv.first.second;
                const auto& fps_vec = kv.second;
                
                std::cout << "  " << width << "x" << height << ": ";
                for (size_t i = 0; i < fps_vec.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << fps_vec[i];
                }
                std::cout << std::endl;
            }

            return 0;
        }

        po::notify(vm);
    } catch (const po::error &e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Usage: " << argv[0] << " [options]\n";
        std::cerr << desc;
        return -1;
    }
    
    if( !validate_frame_properties(frame_width, frame_height, opt_framerate) ||
        !validate_acc_fps(acc_framerate) || 
        !validate_gyro_fps(gyro_framerate) )
        return -1;

    if (!check_imu_is_supported()) {
        std::cerr << "No realsense device with IMU support found. Exiting.\n";
        return EXIT_FAILURE;
    }

    std::cout << "RealSense Record - Asynchronious mode.\n";
    std::cout << "Recording " << dataset_size << " frames in " << data_dir << std::endl;
    std::cout << "Optical FPS: " << opt_framerate << "\nAccelerometer FPS: " << acc_framerate << "\nGyroscope FPS: " << gyro_framerate << "\nImage Width: " << frame_width << "\nImage Height: " << frame_height << std::endl;

    // Setup the database folders and index files
    create_dir_if_not_exists(data_dir);

    // Files for storing the indexes of the rgb, depth, accelerometer and gyroscope frames
    std::ofstream rgb_file;
    std::ofstream depth_file; 
    std::ofstream acc_file;   
    std::ofstream gyr_file;

    rgb_file.open(data_dir + "/rgb.txt", std::ios_base::out);
    if(!rgb_file.is_open()) {
        std::cerr << "Could not open rgb file index. Exiting\n";
        return -1;
    }
    depth_file.open(data_dir + "/depth.txt", std::ios_base::out);
    if(!depth_file.is_open()) {
        std::cerr << "Could not open depth file index. Exiting\n";
        return -1;
    }
    acc_file.open(data_dir + "/acc.txt", std::ios_base::out);
    if(!acc_file.is_open()) {
        std::cerr << "Could not open accelerometer file index. Exiting\n";
        return -1;
    }
    gyr_file.open(data_dir + "/gyr.txt", std::ios_base::out);
    if(!gyr_file.is_open()) {
        std::cerr << "Could not open gyroscope file index. Exiting\n";
        return -1;
    }

    create_dir_if_not_exists(data_dir + "/rgb");
    create_dir_if_not_exists(data_dir + "/depth");
    create_dir_if_not_exists(data_dir + "/acc");
    create_dir_if_not_exists(data_dir + "/gyr");

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    //Structures for indexing the data frames, based on their timestamps
    std::map<double, rs2_vector> gyrs;
    std::map<double, rs2_vector> accs;
    std::map<double, cv::Mat> rgbs;
    std::map<double, cv::Mat> depths;

    // Access mutex
    std::mutex mutex;

    // Load a camera configuration
    rs2::config     cfg;
	rs2::context    ctx;
	auto devices    = ctx.query_devices();
	rs2::device dev = devices[0];

	rs2::pipeline pipe(ctx);

    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	std::string json_file_name = "../configs/high_density.json";
    std::cout << "Configuring camera : " << serial << std::endl;
    
    // Query which device sensors are used to in case you want to configure them
    std::cout << "Active Sensors: " << "(0) " << get_sensor_name(dev.query_sensors()[0]) << " (1) " << get_sensor_name(dev.query_sensors()[1]) << " (2) " << get_sensor_name(dev.query_sensors()[2]) << std::endl;
    
    // Change the RGB and depth autoexposure parameter
    try
    {
        dev.query_sensors()[0].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        dev.query_sensors()[1].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        dev.query_sensors()[0].set_option(rs2_option::RS2_OPTION_EXPOSURE, depth_exposure);
        dev.query_sensors()[1].set_option(rs2_option::RS2_OPTION_EXPOSURE, rgb_exposure);
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Print the autoexposure settings for the RGB and Depth sensor
    std::cout << "Sensor " << get_sensor_name(dev.query_sensors()[0]) << " exposure: " << dev.query_sensors()[0].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
    std::cout << "Sensor " << get_sensor_name(dev.query_sensors()[1]) << " exposure: " << dev.query_sensors()[1].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
	
    // Check if camera supports loading a .json configuration
	if (dev.is<rs400::advanced_mode>() && ( boost::filesystem::exists(json_file_name)) ) {
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
	}
	else {
		std::cout << "Current device doesn't support advanced-mode!\n";
	}

    // Enable the device using the requested formats
    try
    {
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, acc_framerate);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, gyro_framerate);

        cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16,  opt_framerate);    
        cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_RGB8, opt_framerate);
    }
    catch (const rs2::error & e)
    {
        std::cerr << "Error enabling stream. " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    std::cout << "--- Starting pipe ---" << std::endl;
    
    // Align the color stream to the depth stream
    rs2::align align(RS2_STREAM_DEPTH);         
    
    auto start = std::chrono::system_clock::now();

    bool bfirst = false;

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    auto callback = [&](const rs2::frame& frame)
    { 
        // Exit if we reached the requested data size
        if(depths.size() > dataset_size) return;

        // Any modification to common memory should be done under lock
        std::lock_guard<std::mutex> lock(mutex);

        // Implement a simple "starting in 2 seconds ..." procedure
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

        // Cout the recorded data sizes
        std::cout << "Recording " << (int)dataset_size << " (depth) frames " 
            << "Depths: " << depths.size() << " RGBs: " << rgbs.size() 
            << " Accs " << accs.size() << " Gyrs " << gyrs.size() << "\r";//std::endl; 
        
        // Retrieve RGB and depth frames as one frameset.
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {     
            // Retrieve the color frame and align in to the depth stream
            rs2::video_frame color_frame = fs.get_color_frame().apply_filter(align);
            float width  = color_frame.get_width();
            float height = color_frame.get_height();

            // Index the color cv mat with its timestamp
            cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
            rgbs[color_frame.get_timestamp()] = image.clone();

            // Retrieve the depth frame
            rs2::video_frame depth_frame = fs.get_depth_frame();
            float dwidth  = depth_frame.get_width();
            float dheight = depth_frame.get_height();     

            // Index the depth cv mat with its timestamp
            cv::Mat dimage(cv::Size(dwidth, dheight), CV_16UC1, (void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
            depths[depth_frame.get_timestamp()] = dimage.clone();
        }
        else
        {
            // Stream that bypass synchronization (such as IMU) will produce single frames
            rs2::motion_frame motion = frame.as<rs2::motion_frame>();
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                // Get gyro measurement
                double ts = motion.get_timestamp();
                rs2_vector gyro_data = motion.get_motion_data();
                gyrs[ts] = gyro_data;
                
                // Uncomment this to cout the gyro timestamps
                //std::cout << std::fixed << "Gyro frame Timestamp: \t" <<ts << std::endl;
            }
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) 
            {
                // Get accelerometer measurements
                double ts = motion.get_timestamp();
                rs2_vector accel_data = motion.get_motion_data();
                accs[ts] = accel_data;

                // Uncomment this to cout the accelerometer timestamps
                //std::cout << std::fixed << "Acceleration frame Timestamp: \t" << ts << std::endl;
            }
        }
    };

    // Start pipe and load the configuration file    
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    
    // Get the device intrinsics
    auto color_stream       = profiles.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto resolution         = std::make_pair(color_stream.width(), color_stream.height());
    auto intr               = color_stream.get_intrinsics();
    auto principal_point    = std::make_pair(intr.ppx, intr.ppy);
    auto focal_length       = std::make_pair(intr.fx, intr.fy);
    rs2_distortion model    = intr.model;

    auto dep                = profiles.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intr_depth         = dep.get_intrinsics();
    std::cout << std::fixed << std::setprecision(4) << "Depth distortion: [" << intr_depth.coeffs[0] << " " << intr_depth.coeffs[1] << " " << intr_depth.coeffs[2] << " " << intr_depth.coeffs[3] << " " << intr_depth.coeffs[4] << "]" << std::endl;

    // Cout and save the dataset intrinsics into a file
    std::cout << std::fixed << std::setprecision(4) << "Intrinsics: " << "px: " << intr.ppx << " py: " << intr.ppy << " fx: " << intr.fx << " fy: " << intr.fy << std::endl;
    
    std::ofstream fintrinsics  (data_dir + "/rgb.intrinsics", std::ios_base::out);
    fintrinsics << std::setprecision(10) << intr.fx << ", 0.0, " << intr.ppx << std::endl;
    fintrinsics << std::setprecision(10) << "0.0, " << intr.fy << ", " << intr.ppy << std::endl;
    fintrinsics << std::setprecision(10) << "0.0, " << "0.0, " << "1.0" << std::endl;
    
    std::cout << std::fixed << std::setprecision(4) << "Distortion: [" << intr.coeffs[0] << " " << intr.coeffs[1] << " " << intr.coeffs[2] << " " << intr.coeffs[3] << " " << intr.coeffs[4] << "]" << std::endl;
    
    std::ofstream fdistortion  (data_dir + "/rgb.distortion", std::ios_base::out);
    fdistortion << std::setprecision(10) << intr.coeffs[0] << " " << intr.coeffs[1] << " " << intr.coeffs[2] << " " << intr.coeffs[3] << " " << intr.coeffs[4] << std::endl;
    
    // Run the program until we have the requested ammount of frames
    while (depths.size() <= dataset_size)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait for one second to make sure all data have arrived and stop the device
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {
        //Lock the pipe stopping to make sure that no data are left in the callback 
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << std::endl << "Stopping device" << std::endl;
        pipe.stop();
        std::cout << "Stopped" << std::endl;
        BEEP_OFF;   // Notify with sound that the camera stoped recording
    }

    std::cout << std::fixed;

    // Playback the recorded RGBs
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

    // Save the data into files
    int ii = 0;
    for (auto i : rgbs) {
        std::string namergb = std::string("rgb/r") + std::to_string(ii++) + std::string(".png"); 
        rgb_file << std::fixed << i.first << " " << namergb << std::endl;
        cv::Mat locl_rgb;
        cv::cvtColor  (i.second, locl_rgb, cv::COLOR_BGR2RGB);
        cv::imwrite(data_dir + std::string("/") + namergb, locl_rgb);
        std::cout << "Saving RGBs: " << ii << "\r";
    }
    int dd = 0;
    for (auto i : depths) {
        std::string namedepth = std::string("depth/d") + std::to_string(dd++) + std::string(".png"); 
        depth_file << std::fixed << i.first << " " << namedepth << std::endl;
        cv::imwrite(data_dir + std::string("/") + namedepth, depths[i.first]);
        std::cout << "Saving Depths: " << dd << "\r";
    }
    int aa = 0;
    for (auto i : accs) {
        std::string nameacc = std::string("acc/a") + std::to_string(aa++) + std::string(".txt"); 
        acc_file << std::fixed << i.first << " " << nameacc << std::endl;
        std::ofstream acc_  (data_dir + std::string("/") + nameacc, std::ios_base::out);
        acc_ << accs[i.first].x << " " <<  accs[i.first].y << " " << accs[i.first].z << std::endl;
        std::cout << "Saving accels: " << aa << "\r";
    }
    int gg = 0;
    for (auto i : gyrs) {
        std::string namegyr = std::string("gyr/g") + std::to_string(gg++) + std::string(".txt"); 
        gyr_file << std::fixed << i.first << " " << namegyr << std::endl;
        std::ofstream gyr_  (data_dir + std::string("/") + namegyr, std::ios_base::out);
        gyr_ << gyrs[i.first].x << " " <<  gyrs[i.first].y << " " << gyrs[i.first].z << std::endl;
        std::cout << "Saving gyroscopes: " << gg << "\r";
    }
    std::cout << "                                 " << "\r";
    std::cout << "Finished" << "\r" << std::endl;

    return EXIT_SUCCESS;
}