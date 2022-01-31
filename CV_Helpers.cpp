#include "CV_Helpers.h"

void drawPoints(std::vector<cv::Point> &points, cv::Mat &frame, cv::Scalar color) {
  int drawn = 0;
  if( points.size() <= 10 ) return;
  for( unsigned i = points.size() - 1; i>0; i-- ) {
    circle(frame, points[i], 5.0, color, 2, 1);      
    drawn++;
    if(drawn>30) {
      drawn = 0; return; 
    };
  }
}

// IplImage *matToImage(cv::Mat image1) {
//   IplImage* image2;
//   image2 = cvCreateImage(cvSize(image1.cols,image1.rows),8,3);
// //   image2 = cvCreateImage(cvSize(image1.cols,image1.rows),1,1);
//   IplImage ipltemp=image1;
//   cvCopy(&ipltemp,image2);  
//   return image2;
// }

CVVideo::CVVideo() {
}

CVVideo::CVVideo(int source) {
  open(source);
}

bool CVVideo_base::checkEsc() {
  return( (char)waitKey(1) == 27 );
}
bool CVVideo::open(int source) { 
    cap.set(CAP_PROP_FRAME_WIDTH,  1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1024);    
    // cap.set(CV_CAP_PROP_FPS, 30);    
    return cap.open(source); 
}
bool CVVideo::open() {
//     cap.set(CV_CAP_PROP_FRAME_WIDTH,  320);
//     cap.set(CV_CAP_PROP_FRAME_HEIGHT, 320);    
//     cap.set(CV_CAP_PROP_FPS, 10);    
    return cap.open("/dev/video0");
}
Mat CVVideo::getRGB() {
  Mat frame;
  try {
    cap >> frame;
  } catch( cv::Exception ) {
    std::cout << "Exception" << std::endl;
  }    
  return frame;
}

CVVideoLoader::CVVideoLoader(std::string filename)
: video_filename(filename) {
  open();
  fps = ( int ) cap.get(CAP_PROP_FPS );
  std::cout << "FPS is: " << fps << std::endl;
}
bool CVVideoLoader::open() { 
  return cap.open(video_filename); 
}
bool CVVideoLoader::checkEsc(double desired_fps) {
  return ( (char)waitKey(1000.0 / (desired_fps)) == 27 );
}
Mat CVVideoLoader::getRGB() {
  Mat frame;
  cap >> frame;    
  return frame;
}

  CVVideoWriter::CVVideoWriter(std::string filename, cv::Size size) 
  : _size(size),
//    CV_FOURCC('D','I','V','X') {MJPG    ('M','J','P','G')
    _writer(filename.c_str(), cv::VideoWriter::fourcc('M','J','P','G'), 40, size, true) {
  }
  CVVideoWriter::CVVideoWriter(std::string filename, CVVideo &input_cap)
  : _writer(filename.c_str(), cv::VideoWriter::fourcc('M','J','P','G'),
	    input_cap.cap.get(CAP_PROP_FPS),
	    cv::Size(input_cap.cap.get(CAP_PROP_FRAME_WIDTH),
		     input_cap.cap.get(CAP_PROP_FRAME_HEIGHT))),
    _size(cv::Size(input_cap.cap.get(CAP_PROP_FRAME_WIDTH),
		 input_cap.cap.get(CAP_PROP_FRAME_HEIGHT))) {
  }
//   CVVideoWriter(CVOpenNIAsus &input_cap)
//   : _writer("test.avi", CV_FOURCC('D','I','V','X'),
// 	    120
// 	    cv::Size(input_cap.cap.get(CV_CAP_PROP_FRAME_WIDTH),
// 	    input_cap.cap.get(CV_CAP_PROP_FRAME_HEIGHT))){
//   }
    
  void CVVideoWriter::write(cv::Mat &frame) {
    cv::Mat frame_local = frame;
    cv::resize(frame_local,frame_local,_size);
   _writer.write(frame_local); 
  }
  void CVVideoWriter::close() {
    _writer.release();
  }
  CVVideoWriter::~CVVideoWriter() {
    _writer.release();
  }

bool CVOpenNI::open() {     
  cap.open( CAP_OPENNI_ASUS/*CAP_OPENNI2*/ );
  if( !cap.isOpened() )
      cap.open( CAP_OPENNI );    
  //cap.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_QVGA_60HZ );
  return cap.isOpened();
}
Mat CVOpenNI::getRGB() {
  cap.grab();
  Mat frame;
  cap.retrieve( frame, CAP_OPENNI_BGR_IMAGE );  
//     imshow( "rgb image", frame);
  return frame;
}

Mat CVOpenNI::getDepthmap(double threshold) {       
  Mat depthMap;
  cap.grab();
  cap.retrieve( depthMap, CAP_OPENNI_DEPTH_MAP );  
  
  Mat show; 
  depthMap.convertTo( show, CV_8UC1, threshold );
  
  return show;
}

bool CVOpenNIAsus::open() {       
  openni::OpenNI::initialize();

  openni::Status err;
  err = device.open		(openni::ANY_DEVICE); 		CHECK_ERR(err, "Open device");
  err = ir_rgb.create		(device, openni::SENSOR_COLOR);	CHECK_ERR(err, "Create color stream");
  openni::VideoMode mMode;
  mMode.setResolution		( 1280, 1024 );
  // mMode.setFps			( 10 );
  mMode.setPixelFormat		( openni::PIXEL_FORMAT_RGB888 );  
  err = ir_rgb.setVideoMode	(mMode);  			CHECK_ERR(err, "Set video mode");
  err = ir_rgb.start();  					CHECK_ERR(err, "Starting color stream");
  
  err = ir_depth.create(device, openni::SENSOR_DEPTH);		CHECK_ERR(err, "Creating the depth stream");
  err = ir_depth.start();      					CHECK_ERR(err, "Starting the depth stream");

//   err = device.setDepthColorSyncEnabled(true);
  if( device.isImageRegistrationModeSupported( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) ) {
      err = device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
      CHECK_ERR(err, "Setting registration for RGB-Depth");
  }
  ir_rgb.setMirroringEnabled(false);

  return (err==openni::STATUS_OK);
}

/**
 * @function getRGB
 * @details Reads the RGB part of a frame from an RGBD camera.
 */
Mat CVOpenNIAsus::getRGB() {  
  Mat frame;
  openni::Status err = ir_rgb.readFrame(&irf_rgb);
  CHECK_ERR(err, "Read color frame");
  const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)irf_rgb.getData();
  frame.create(irf_rgb.getHeight(), irf_rgb.getWidth(), CV_8UC3);
  memcpy( frame.data, imageBuffer, 3*irf_rgb.getHeight()*irf_rgb.getWidth()*sizeof(uint8_t) );
  cv::cvtColor(frame,frame,COLOR_BGR2RGB); //this will put colors right    

  return frame;
}

Mat CVOpenNIAsus::getDepthmap(double threshold) {         
  openni::Status err;
  if(device.getSensorInfo(openni::SENSOR_DEPTH) != NULL) {
      err = ir_depth.readFrame(&irf_depth);        
      CHECK_ERR(err, "Read depth frame");
  } 
  Mat mat_cloud(1, irf_depth.getHeight()*irf_depth.getWidth(), CV_32FC3);
  Point3f* data = mat_cloud.ptr<cv::Point3f>();  

  if(!irf_depth.isValid()) std::cout << "Frame is invalid" << std::endl;

  openni::VideoMode 	depthVideoMode 	= ir_depth.getVideoMode();
  int 			depthWidth 	= depthVideoMode.getResolutionX();
  int 			depthHeight	= depthVideoMode.getResolutionY();	
      
  int 			buf_size 	= irf_depth.getDataSize();
  openni::SensorType 	s_type 		= irf_depth.getSensorType();

  openni::DepthPixel* 	pDepth 		= (openni::DepthPixel*)irf_depth.getData();
  const uint16_t* 	imgBuf 		= (const uint16_t*)irf_depth.getData();
  
  cv::Mat 		DethBuf;

  DethBuf.create(depthHeight, depthWidth, CV_16U);
  memcpy(DethBuf.data, imgBuf, depthHeight * depthWidth * sizeof(uint16_t));
  DethBuf.convertTo(DethBuf, CV_16U);     /*CV_8U*/
  
  return DethBuf;
}

CVOpenNIAsus::~CVOpenNIAsus() {
  ir_depth.stop();;
  ir_depth.destroy();  
  ir_rgb.stop();;
  ir_rgb.destroy();
  device.close();
  openni::OpenNI::shutdown();    
}
