/* sample listener for realsense_record_publisher */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

class ImageListener : public rclcpp::Node
{
public:
  explicit ImageListener(const rclcpp::NodeOptions& options) : Node("image_listener", options)
  {
    // set QoS
    rclcpp::QoS qos(1);  // depth 1 is typical for images
    //qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    // Declare parameters
    this->declare_parameter<std::string>("rgb_image_topic_name", "/camera/color/image_raw");
    this->declare_parameter<std::string>("rgb_info_topic_name", "/camera/color/camera_info");
    this->declare_parameter<std::string>("depth_image_topic_name", "/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("depth_info_topic_name", "/camera/aligned_depth_to_color/camera_info");

    // Get values
    _rgb_image_topic_name = this->get_parameter("rgb_image_topic_name").as_string();
    _rgb_info_topic_name = this->get_parameter("rgb_info_topic_name").as_string();
    _depth_image_topic_name = this->get_parameter("depth_image_topic_name").as_string();
    _depth_info_topic_name = this->get_parameter("depth_info_topic_name").as_string();

    // Subscribe to RGB image and camera info
    _rgb_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      _rgb_image_topic_name, qos,
      std::bind(&ImageListener::rgbImageCallback, this, std::placeholders::_1));

    _rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      _rgb_info_topic_name, qos,
      std::bind(&ImageListener::rgbInfoCallback, this, std::placeholders::_1));

    // Subscribe to depth image and camera info
    _depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      _depth_image_topic_name, qos,
      std::bind(&ImageListener::depthImageCallback, this, std::placeholders::_1));

    _depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      _depth_info_topic_name, qos,
      std::bind(&ImageListener::depthInfoCallback, this, std::placeholders::_1));
  }

private:
  void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      auto cv_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::imshow("RGB Image", cv_image->image);
      cv::waitKey(1);
    
      //cv::imwrite("/tmp/rgb.png", cv_image->image);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void rgbInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Received RGB camera info");
    //printInfo(msg);
  }

  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      auto cv_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv::imshow("Depth Image", cv_image->image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Received depth camera info");
    //printInfo(msg);
  }

  // Print camera info
  void printInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "  Width: %d", msg->width);
    RCLCPP_INFO(this->get_logger(), "  Height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "  Distortion model: %s", msg->distortion_model.c_str());
    RCLCPP_INFO(this->get_logger(), "  D: [%s]",
    std::accumulate(msg->d.begin(), msg->d.end(), std::string(""),
      [](const std::string& a, double b) {
        return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b);
    }).c_str());
    RCLCPP_INFO(this->get_logger(), "  K (Intrinsic camera matrix): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
              msg->k[0], msg->k[1], msg->k[2],
              msg->k[3], msg->k[4], msg->k[5],
              msg->k[6], msg->k[7], msg->k[8]);
    RCLCPP_INFO(this->get_logger(), "  R (Rectification matrix): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
              msg->r[0], msg->r[1], msg->r[2],
              msg->r[3], msg->r[4], msg->r[5],
              msg->r[6], msg->r[7], msg->r[8]);
    RCLCPP_INFO(this->get_logger(), "  P (Projection/camera matrix): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
              msg->p[0], msg->p[1], msg->p[2], msg->p[3],
              msg->p[4], msg->p[5], msg->p[6], msg->p[7],
              msg->p[8], msg->p[9], msg->p[10], msg->p[11]);
    RCLCPP_INFO(this->get_logger(), "  Binning: x=%d, y=%d", msg->binning_x, msg->binning_y);
    RCLCPP_INFO(this->get_logger(), "  ROI: x_offset=%d, y_offset=%d, height=%d, width=%d, do_rectify=%s",
              msg->roi.x_offset, msg->roi.y_offset,
              msg->roi.height, msg->roi.width,
              msg->roi.do_rectify ? "true" : "false");
  }

  std::string _rgb_image_topic_name;
  std::string _rgb_info_topic_name;
  std::string _depth_image_topic_name;
  std::string _depth_info_topic_name;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _rgb_image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _rgb_info_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depth_image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _depth_info_sub;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<ImageListener>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

