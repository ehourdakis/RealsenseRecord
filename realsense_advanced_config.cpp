#include <librealsense2/rs.hpp>                // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>  // Include RealSense Cross Platform API
#include <iostream>
#include "roboslam_realsense.h"
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <fstream>
using namespace rs2;
using namespace std;

int main(int argc, char* argv[]) try
{
	rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
	// Create a simple OpenGL window for rendering:
	window app(640, 360, "RealSense Json Example");

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	// Declare rates printer for showing streaming rates of the enabled streams.
	rs2::rates_printer printer;

	// Create a Pipeline - this serves as a top-level API for streaming and processing frames

	rs2::context ctx; //declare context object in main function
	//get device
	auto devices = ctx.query_devices();
	rs2::device dev = devices[0];
	rs2::pipeline p(ctx);
	rs2::config cfg;
	std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	std::string json_file_name = "/home/manos/Desktop/wrong.json"; //use your own filename here

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

	cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 360, rs2_format::RS2_FORMAT_Z16, 90);
    // cfg.enable_stream(rs2_stream::RS2_STREAM_INFRARED, 848, 480, rs2_format::RS2_FORMAT_Y8, 90);
	cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 640, 360, rs2_format::RS2_FORMAT_RGB8, 90);

	// Start pipeline with chosen configuration
	p.start(cfg);

	while (app) // Application still alive?
	{
		rs2::frameset data = p.wait_for_frames().    // Wait for next set of frames from the camera
			apply_filter(printer).     // Print each enabled stream frame rate
			apply_filter(color_map);   // Find and colorize the depth data

// The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
// Each texture is displayed on different viewport according to it's stream unique id
		app.show(data);
	}

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