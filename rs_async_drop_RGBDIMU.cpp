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
#include <algorithm>  // for std::find
#include <boost/program_options.hpp> // command line argument options
#include <boost/algorithm/string.hpp>
#include <sys/ioctl.h> // for FIONREAD
#include <termios.h>  // for struct termios, tcgetattr, ICANON

// validate the value of the accelerometer fps
bool validate_acc_fps(const std::vector<int>& allowed_values, const int option_value) {
  if (std::find(allowed_values.begin(), allowed_values.end(), option_value) == allowed_values.end()) {
    std::cout << "Invalid accelerometer fps " << option_value << ". Allowed values are: ";
    for (int val : allowed_values) {
        std::cout << val << " ";
    }
    std::cout << "\n";
    return false;
  }
  return true;
}

// validate the value of the gyroscope fps
bool validate_gyro_fps(const std::vector<int>& allowed_values, const int option_value) {
  if (std::find(allowed_values.begin(), allowed_values.end(), option_value) == allowed_values.end()) {
    std::cout << "Invalid gyro fps " << option_value << ". Allowed values are: ";
    for (int val : allowed_values) {
        std::cout << val << " ";
    }
    std::cout << "\n";
    return false;
  }
  return true;
}

// validate whether the combination frame size / fps exists
bool validate_frame_properties(const std::map<std::pair<int, int>, std::vector<int> >& res_and_fps,
    int frame_width, int frame_height, int opt_framerate) {

    auto it = res_and_fps.find({frame_width, frame_height});
    if (it == res_and_fps.end()) {
        std::cout << "Invalid frame resolution: " << std::to_string(frame_width) << "x" << std::to_string(frame_height) << std::endl;
        std::cout << "Available resolutions are: " << std::endl;
        for (auto f : res_and_fps) {
            std::cout << "  " << f.first.first << "x" << f.first.second << std::endl;
        }

        return false;
    }

    int fps = opt_framerate;
    auto fps_vec = it->second;
    if (std::find(fps_vec.begin(), fps_vec.end(), fps) == fps_vec.end()) {
        std::cout << "Invalid frame rate for resolution " << frame_width << "x" << std::to_string(frame_height) << ": " << std::to_string(opt_framerate) << std::endl;
        std::cout << "Available framerates for " << frame_width << "x" << frame_height << ":";
        for (auto f : fps_vec) {
            std::cout << " " << f;
        }
        std::cout << " (Hz)" << std::endl;

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

namespace po = boost::program_options;

int main(int argc, char * argv[]) 
{
    // Default application parameters 
    std::string data_dir = "/home/";
    std::string dev_serial = "";
    int dataset_size = 500;
    int opt_framerate = 90;
    int frame_width = 640;
    int frame_height = 360;
    int acc_framerate = 255;
    int gyro_framerate = 400;
    int rgb_exposure = 200;
    int depth_exposure = 10;
    int laser_power = 160;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Help message")
        ("dataset_dir", po::value<std::string>(&data_dir)->required(), "Directory to save the recorded dataset")
        ("dev_serial", po::value<std::string>(&dev_serial)->default_value(""), "Serial number of RS device to use")
        ("dataset_size", po::value<int>(&dataset_size), "Size of the recorded dataset")
        ("rgb_fps", po::value<int>(&opt_framerate)->default_value(60), "Depth frame rate")
        ("frame_width", po::value<int>(&frame_width)->default_value(640), "Frame width")
        ("frame_height", po::value<int>(&frame_height)->default_value(360), "Frame height")
        ("acc_framerate", po::value<int>(&acc_framerate)->default_value(200), "Accelerometer framerate")
        ("gyro_framerate", po::value<int>(&gyro_framerate)->default_value(400), "Gyroscope framerate")
        ("rgb_exposure", po::value<int>(&rgb_exposure)->default_value(200), "RGB exposure")
        ("depth_exposure", po::value<int>(&depth_exposure)->default_value(10), "Depth exposure")
        ("laser_power", po::value<int>(&laser_power)->default_value(160), "Laser power");

    po::positional_options_description p;
    p.add("dataset_dir", 1)
     .add("dataset_size", 1)
     .add("rgb_fps", 1)
     .add("frame_width", 1)
     .add("frame_height", 1)
     .add("acc_framerate", 1)
     .add("gyro_framerate", 1)
     .add("rgb_exposure", 1)
     .add("depth_exposure", 1)
     .add("laser_power", 1);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

        po::notify(vm);
    } catch (const po::error &e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Usage: " << argv[0] << " [options]\n";
        std::cerr << desc;
        return -1;
    }

    //rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cout << "No RealSense devices connected." << std::endl;
        return EXIT_FAILURE;
    }

    int devidx = -1;
    if (!dev_serial.empty()) {
        // Iterate over the detected RS cameras.
        for (size_t i = 0; i < devices.size(); ++i) {
            if (devices[i].supports(RS2_CAMERA_INFO_SERIAL_NUMBER) && dev_serial == devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
                devidx = i;
		break;
            }
        }
    }
    else {
        // Use first detected camera.
        devidx = 0;
    }
    if (devidx == -1) {
        std::cout << "No RealSense device with serial " << dev_serial << " found." << std::endl;
        return EXIT_FAILURE;
    }

    const rs2::device& dev = devices[devidx];
    const std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    // Tables of acceptable frame size / fps pairs for the RS camera.
    std::map<std::pair<int, int>, std::vector<int>> rgb_res_and_fps, depth_res_and_fps;
    std::pair<std::vector<int>, std::vector<int>> accel_and_gyro;
    int dpth_idx, rgb_idx;
    retrieve_res_and_rate_combos(dev, RS2_FORMAT_RGB8, RS2_FORMAT_Z16,
                    rgb_res_and_fps, rgb_idx, depth_res_and_fps, dpth_idx, accel_and_gyro);

        if (vm.count("help")) {
            std::cout << "RealsenseRecord. Record sensor data from a realsense camera" << std::endl << std::endl;
            std::cout << "Run as: " << std::endl;
            std::cout << argv[0] << " data/ 1000 30" << std::endl << std::endl << std::endl;

            std::cout << desc << "\n";

            std::cout << "Available camera RGB frame size & frame rates:" << std::endl;
            for (const auto& kv : rgb_res_and_fps) {
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

            std::cout << "Available camera Depth frame size & frame rates:" << std::endl;
            for (const auto& kv : depth_res_and_fps) {
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


    if( !validate_frame_properties(rgb_res_and_fps, frame_width, frame_height, opt_framerate) ||
        !validate_frame_properties(depth_res_and_fps, frame_width, frame_height, opt_framerate) ||
        !validate_acc_fps(accel_and_gyro.first, acc_framerate) ||
        !validate_gyro_fps(accel_and_gyro.second, gyro_framerate) )
        return -1;

    if (!check_imu_is_supported()) {
        std::cerr << "No realsense device with IMU support found. Exiting.\n";
        return EXIT_FAILURE;
    }

    std::cout << "RealSense Record - Asynchronous mode.\n";
    std::cout << "Recording in " << data_dir << std::endl;
    std::cout << "Optical FPS: " << opt_framerate << "\nImage Width: " << frame_width << "\nImage Height: " << frame_height << "\nAccelerometer FPS: " << acc_framerate << "\nGyroscope FPS: " << gyro_framerate << std::endl;

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

    // Structures for indexing the data frames, based on their timestamps
    std::map<double, rs2_vector> gyrs;
    std::map<double, rs2_vector> accs;
    std::map<double, cv::Mat> rgbs;
    std::map<double, cv::Mat> depths;

    // Access mutex
    std::mutex mutex;

    std::string json_file_name = "../configs/high_density.json";
    std::cout << "Configuring camera: " << serial << std::endl;
    
    // Query which device sensors are used to in case you want to configure them
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    std::cout << "Active Sensors: " << "(0) -> " << get_sensor_name(sensors[0]) << ", (1) -> " << get_sensor_name(sensors[1]) << ", (2) -> " << get_sensor_name(sensors[2]) << std::endl;
    // This check is redundant but does no harm
    if (dpth_idx < 0 || rgb_idx < 0) {
        std::cerr << "Failed to find depth or RGB sensor!" << std::endl;
        return EXIT_FAILURE;
    }

    // Change the RGB / depth auto exposure and the laser power parameters
    try
    {
        auto dpth_rng = sensors[dpth_idx].get_option_range(rs2_option::RS2_OPTION_EXPOSURE);
        float clipped_depth_exposure = std::min(std::max(static_cast<float>(depth_exposure), dpth_rng.min), dpth_rng.max);
        if (std::fabs(depth_exposure - clipped_depth_exposure) > 0.1f) {
		std::cerr << "Depth exposure clipped to " << clipped_depth_exposure << std::endl;
        }

        auto rgb_rng = sensors[rgb_idx].get_option_range(rs2_option::RS2_OPTION_EXPOSURE);
        float clipped_rgb_exposure = std::min(std::max(static_cast<float>(rgb_exposure), rgb_rng.min), rgb_rng.max);
        if (std::fabs(rgb_exposure - clipped_rgb_exposure) > 0.1f) {
		std::cerr << "RGB exposure clipped to " << clipped_rgb_exposure << std::endl;
        }

        auto pwr_rng = sensors[dpth_idx].get_option_range(rs2_option::RS2_OPTION_LASER_POWER);
        float clipped_laser_power = std::min(std::max(static_cast<float>(laser_power), pwr_rng.min), pwr_rng.max);
        if (std::fabs(laser_power - clipped_laser_power) > 0.1f) {
		std::cerr << "Laser power clipped to " << clipped_laser_power << std::endl;
        }

        sensors[dpth_idx].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        sensors[rgb_idx].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        sensors[dpth_idx].set_option(rs2_option::RS2_OPTION_EXPOSURE, clipped_depth_exposure);
        sensors[rgb_idx].set_option(rs2_option::RS2_OPTION_EXPOSURE, clipped_rgb_exposure);
        sensors[dpth_idx].set_option(rs2_option::RS2_OPTION_LASER_POWER, clipped_laser_power);
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Print the configured settings for the RGB and Depth sensor
    std::cout << "Sensor " << get_sensor_name(sensors[dpth_idx]) << " exposure: " << sensors[dpth_idx].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
    std::cout << "Sensor " << get_sensor_name(sensors[dpth_idx]) << " power: " << sensors[dpth_idx].get_option(rs2_option::RS2_OPTION_LASER_POWER) << std::endl;
    std::cout << "Sensor " << get_sensor_name(sensors[rgb_idx]) << " exposure: " << sensors[rgb_idx].get_option(rs2_option::RS2_OPTION_EXPOSURE) << std::endl;
	
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
    rs2::config cfg;
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
    auto bUserExit = false;

    // callback that checks user input for the escape button or dataset end
    auto do_record = [&]() {
        // Stop if ESC pressed, record until dataset_size limit reached (if > 0)
        return !bUserExit && ((dataset_size <= 0) || (depths.size() < static_cast<size_t>(dataset_size)));
    };

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    auto callback = [&](const rs2::frame& frame)
    { 
        // Exit if we reached the requested data size
        if(do_record()==false) return;

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
            if(ts >= wait_time) 
            {
                bfirst = true;
                std::cout << "Recording ";
                
                if(dataset_size > 0)
                    std::cout << std::to_string(dataset_size).c_str() << '\0';
                else 
                    std::cout << "until Esc is pressed" << '\0';
                
                std::cout << "                     " << std::endl; // remove trailing characters from previous cout
    
                BEEP_ON;
            }

            return;
        };

        // Cout the recorded data sizes
        std::cout << "Depths: " << depths.size() << " RGBs: " << rgbs.size() 
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
    rs2::pipeline pipe(ctx);

    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    
    // Get the device intrinsics
    auto color_stream       = profiles.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intr               = color_stream.get_intrinsics();

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
    while (do_record())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Check for keyboard input
        if(kbhit()) {
            int ch = getchar();
            // If Escape key is pressed
            if (ch == 27) {
                std::cout << "Escape key pressed, stopping recording...                     \n";
                bUserExit = true;
                // break;  // Exit the loop, stopping the recording
            }
        }
    }

    // Wait for one second to make sure all data have arrived and stop the device
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {
        // Lock the pipe stopping to make sure that no data are left in the callback
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << std::endl << "Stopping device" << std::endl;
        pipe.stop();
        std::cout << "Stopped with " <<  depths.size() << " frames" << std::endl;
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
