#ifndef CV_HELPERS
#define CV_HELPERS

#include "opencv2/opencv.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
#include <OpenNI.h>

void drawPoints(std::vector<cv::Point> &points, cv::Mat &frame, cv::Scalar color);
//IplImage *matToImage(cv::Mat image1);

#define CHECK_ERR(m, s) if(m!=openni::STATUS_OK) std::cout<<s<<std::endl; else 
/**
 * @class CVVideo_base
 * @brief Ahstract class wrapper for OpenCVs VideoCapture module.
 * Get frame functions must be implimented by the descendants.
 * 
 * @version 1.0 
 * @details Use this class to handle the data coming from an RGB camera.
 */
class CVVideo_base {  
public:
  bool checkEsc();   
  
  virtual bool 	open		(int source = 0)   = 0;
  virtual Mat 	getRGB		() = 0;
};

/**
 * @class CVVideo
 * @brief A wrapper class for OpenCV's video capture module.
 * This one grabs frames from the camera
 * @version 1.0
 * 
 * @details Use this class to handle the data coming from an RGB camera.
 */
class CVVideo : public CVVideo_base {
public:
  VideoCapture cap;
public:  
  CVVideo			();
  CVVideo			(int source);

  virtual bool open		(int source) ;
  virtual bool open		();
  virtual Mat getRGB		();  
  bool isOpened			() {return true;};
};
/**
 * @class CVVideoLoader
 * @brief A wrapper class for OpenCVs VideoCapture, for
 * loading a video from a filename. 
 */	
class CVVideoLoader : public CVVideo {
public:
  std::string video_filename;
  int fps;
public:  
  CVVideoLoader(std::string filename);
  virtual bool  open() ;
  virtual Mat   getRGB() ;  
  virtual bool  checkEsc(double desired_fps=30.0);
};

/**
 * @class CVVideoWriter
 * @brief Ahstract class wrapper for OpenCVs VideoCapture module.
 * Get frame functions must be implimented by the descendants.
 * 
 * @version 1.0 
 * @details Use this class to handle the data coming from an RGB camera.
 */
class CVVideoWriter {
public:
  VideoWriter _writer;
public:
  cv::Size _size;
  
  CVVideoWriter(std::string filename, cv::Size size) ;
  CVVideoWriter(std::string filename, CVVideo &input_cap);
    
  void write(cv::Mat &frame);
  void close() ;
  ~CVVideoWriter();
};

/**
 * @class CVOpenNI
 * @brief A wrapper class for OpenCV's video capture module for OpenNI
 * The class provides both depth and rgb information.
 * 
 * @version 1.0 
 * @details Use this class to handle the data coming from an AsusXtrion camera.
 */	
class CVOpenNI : public CVVideo {
public:  
  virtual bool 	open() ;
  virtual Mat 	getRGB();
  virtual Mat 	getDepthmap(double threshold=0.05) ;    
};	

/**
 * @class CVOpenNIAsus
 * @brief A wrapper class for OpenCV's video capture module for OpenNI2,
 * used to capture data from Asus Xtion. This is the prefered mode for 
 * retrieving data from Asus OpenNI.
 * 
 * @version 1.0 
 * @details Use this class to handle the data coming from an AsusXtrion camera.
 */
class CVOpenNIAsus : public CVVideo {//CVVideo_base {
public:  
  openni::Device 	device;
  
  openni::VideoStream 	ir_rgb;
  openni::VideoStream 	ir_depth;
  openni::VideoFrameRef irf_rgb;
  openni::VideoFrameRef irf_depth;
  // viz::Viz3d myWindow;
public:
  virtual bool 	open();
  virtual Mat 	getRGB();
  virtual Mat 	getDepthmap(double threshold);    
  
  virtual bool 	open(int i) { return open(); };
  bool isOpened() {return true;};
  
  CVOpenNIAsus& operator=(const CVOpenNIAsus& other) { open(); return *this;}; //fake operator for setting in calibration
  
  ~CVOpenNIAsus();
};	

#endif