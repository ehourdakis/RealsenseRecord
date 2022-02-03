// The code below implements a recorder that retrieves the frame data using the pipelining
// provided by realsense. As a result, it can take advantage of the different filtering 
// methods of the camera. 
// Frames are packed as one RGB + one Depth + multiple IMU acc,rot readings (in between)
// In the current implementation, gyro measurements are downsampled to the frame rate of the 
// accelerometer.

#include "helpers.h"        // Basic data structures for holding frames

#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <fstream>

// A very basic logging 
#define LOGs std::cout 

// Application parameters (parsed from cmd line arguments)
double      data_size = 500;
std::string data_dir  = "/home/";

/*
 * A structure that holds one RGB, one Depth, and multiple acceleration,gyro frames. 
 */
class RGBDAccRotFrame {
public:
    frmRGB                  _rgb;
    frmDepth                _depth;
    std::vector<frmAcc>     _accs;
    std::vector<frmGyro>    _gyros;
};

// Hold all the recorded data in the vector below
vector<RGBDAccRotFrame> All_Recorded_Data;

// Files for storing the indexes of the rgb, depth and imu frames
std::ofstream rgb_file; 
std::ofstream depth_file;
std::ofstream imu_file;

// Save the data using the TUM format
void saveFrameTUMFormatRGBDAccsGyros(RGBDAccRotFrame pair, int iframe) {
    // Save the data in the corresponding folders, using the frame's index
	char namergb[256]; sprintf(namergb, "%s/rgb/r%d.png",   data_dir.c_str(), iframe);
	char namedep[256]; sprintf(namedep, "%s/depth/d%d.png", data_dir.c_str(), iframe);
	char nameimu[256]; sprintf(nameimu, "%s/imu/i%d.csv",   data_dir.c_str(), iframe);

    // Retrieved color frames use the BGR format so we change it to RGB
    cv::Mat locl_rgb;
    cv::cvtColor  (pair._rgb._m, locl_rgb, cv::COLOR_BGR2RGB);

    // Show the new colored image
    cv::imshow("rgb", locl_rgb);
    cv::waitKey(1);
 
    // Save the (new) rgb and depth images
	cv::imwrite(namergb, locl_rgb);
	cv::imwrite(namedep, pair._depth._m);

    // This holds the formated data, from the Eigen matrix
    std::ofstream imu_data  (nameimu, std::ios_base::out);
    // Format the data
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    
    //Saves the IMU measurements (acceleration(3), rotation(3), timestamp(1)) 
    for(unsigned i = 0; i < pair._accs.size(); i++ ) {
        Eigen::Matrix<double, 1, 7> accrot;
        accrot << pair._accs[i]._m.x, pair._accs[i]._m.y, pair._accs[i]._m.z, pair._gyros[i]._m.x, pair._gyros[i]._m.y, pair._gyros[i]._m.z, pair._accs[i]._ts;
        imu_data <<  accrot.format(CSVFormat) << std::endl;
    }

    // Save the index of each frame
    LOGs << "Saving frame: " << iframe << " with " << pair._accs.size() << " accs and " << pair._gyros.size() << " gyros ";
    depth_file    << pair._depth._ts << " depth/d" << iframe << ".png" << std::endl;
    rgb_file	  << pair._rgb._ts   << " rimugb/r"   << iframe << ".png" << std::endl;
    imu_file	  << pair._rgb._ts   << " /i"   << iframe << ".csv" << std::endl;
}

int main(int argc, char * argv[]) 
try
{
    if(argc < 3) { LOGs << "Not enough parameters. Please run as: rs_async_RGBDAccRot_synced $DATASET_DIRECTORY $DATASET_SIZE" << std::endl; return 0; }
    int dataset_size = 100;

    if(argc > 1) data_dir = argv[1];
    if(argc > 2) dataset_size = atoi(argv[2]);
    LOGs << "Recording " << dataset_size << " frames in " << data_dir << std::endl;

    // Setup the database folders and index files
    create_dir_if_not_exists(data_dir);

    rgb_file  .open(data_dir + "/rgb.txt",      std::ios_base::out);
    depth_file.open(data_dir + "/depth.txt",    std::ios_base::out);
    imu_file  .open(data_dir + "/imu.txt",      std::ios_base::out);

    create_dir_if_not_exists(data_dir + "/rgb");
    create_dir_if_not_exists(data_dir + "/depth");
    create_dir_if_not_exists(data_dir + "/imu");

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    //Structures for indexing the data frames, based on their timestamps
    std::map<double, rs2_vector>   gyros;
    std::map<double, rs2_vector>   accs;
    std::map<double, cv::Mat>      rgbs;
    std::map<double, cv::Mat>      depths;

    // Store the multiple acceleration and gyro frames for every RGB-D camera frame
    std::vector<frmAcc>         frame_accs;        
    std::vector<frmGyro>        frame_gyros;
    
    // Used to downsample the gyroscope, by associating only this 
    // measurement with the last accelerometer measurement
    frmGyro                     last_gyro;      

    std::mutex                  mutex;

    if (!check_imu_is_supported()) {
        std::cerr << "No realsense device with IMU support found";
        return EXIT_FAILURE;
    }

    // Load a camera configuration
    rs2::config     cfg;
	rs2::context    ctx;
	auto devices    = ctx.query_devices();
	rs2::device dev = devices[0];

	rs2::pipeline pipe(ctx);
	
    std::string serial          = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	std::string json_file_name  = "../configs/high_density.json";
	std::cout << "Configuring camera : " << serial << std::endl;

    // Check if camera supports loading a .json configuration
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
	}
	else {
		std::cout << "Current device doesn't support advanced-mode!\n";
		return EXIT_FAILURE;
	}

    // Enable the device using the requested formats
	cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_ACCEL, 		                RS2_FORMAT_MOTION_XYZ32F,   250);
    cfg.enable_stream(RS2_STREAM_GYRO, 		                    RS2_FORMAT_MOTION_XYZ32F,   400);
    cfg.enable_stream(RS2_STREAM_DEPTH,         640,  360,      RS2_FORMAT_Z16,             90);    
    cfg.enable_stream(RS2_STREAM_COLOR,         640,  360, 	    RS2_FORMAT_RGB8, 	        90);

    std::cout << "Starting pipe" << std::endl;
    // Align the color stream to the depth stream
    rs2::align align(RS2_STREAM_DEPTH);         

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    auto callback = [&](const rs2::frame& frame)
    {
        // Exit if we reached the requested data size
        if(All_Recorded_Data.size() > dataset_size) return;

        // Any modification to common memory should be done under lock
        std::lock_guard<std::mutex> lock(mutex);

        // With callbacks, all synchronized stream will arrive in a single frameset
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            //TODO: Does filtering affect timestamping and frame rate?
            // fs = fs.apply_filter(align);

            // Retrieve the color frame and align in to the depth stream
            rs2::video_frame color_frame    = fs.get_color_frame().apply_filter(align);
            float width                     = color_frame.get_width();
            float height                    = color_frame.get_height();
        
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

            // Add new group frame (1 paired RGB and Depth, and multiple accelerometer, gyro measurements)
            RGBDAccRotFrame pair;
            pair._rgb   = frmRGB  (color_frame.get_timestamp(), image);
            pair._depth = frmDepth(depth_frame.get_timestamp(), dimage);
            pair._gyros = frame_gyros;
            pair._accs  = frame_accs;
            All_Recorded_Data.push_back(pair);
            std::cout << "-------------------- Saved Frame: " << All_Recorded_Data.size() << std::endl;

            frame_accs.clear();
            frame_gyros.clear();
        }
        else {
            // Stream that bypass synchronization (such as IMU) will produce single frames
            rs2::motion_frame motion = frame.as<rs2::motion_frame>();
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get gyro measurement
                double ts = motion.get_timestamp();
                rs2_vector gyro_data = motion.get_motion_data();
                gyros[ts] = gyro_data;

                // Create a gyro frame and save it as last gyro so that it will be paired with the 
                // next accelerometer frame
                last_gyro = frmGyro(ts, gyro_data);
                std::cout << std::fixed << "Gyro frame Timestamp: \t" << ts << std::endl;
            }
            //The gyro stream is faster, so we insert a pair acc, gyro when the acc frame comes (using the last gyro measured)
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get accelerometer measurements
                double ts = motion.get_timestamp();
                rs2_vector accel_data = motion.get_motion_data();
                accs[ts] = accel_data;

                frame_accs.push_back(frmAcc(ts, accel_data));
                frame_gyros.push_back(last_gyro);

                std::cout << std::fixed << "Acceleration frame Timestamp: \t" << ts << std::endl;
            }
        }
    };

    // Start pipe and load the configuration file
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    std::cout << "RealSense recorder from pipeline" << std::endl << std::endl;

    // Run the program until we have the requested ammount of frames
    while (All_Recorded_Data.size() < dataset_size)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::lock_guard<std::mutex> lock(mutex);
    }

    // Save the data afer all frames have been collected
    std::cout <<"Saving " << All_Recorded_Data.size() << " frames." << std::endl;          
    for(unsigned i = 0; i < All_Recorded_Data.size(); i++ ) 
        saveFrameTUMFormatRGBDAccsGyros(All_Recorded_Data[i], i);   

    // Stop the camera
    pipe.stop();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}