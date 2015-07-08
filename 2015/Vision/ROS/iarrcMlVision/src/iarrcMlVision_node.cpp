// Try quotes if this doesn't work
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Get message prototype for sending to the Arduino managing node
#include <car_serial_comms/ThrottleAndSteering.h>

static const std::string WINDOW = "Subscribed Image";

class ImageProcessor
{
  // Member variables
  ros::NodeHandle nh_;
  // image_transport::ImageTransport it_;
  // image_transport::Subscriber it_sub_;

  ros::Publisher cmd_pub_;
  ros::Subscriber img_sub_; // This is very bad practice, but we're stuck
                              // with what the raspicam_node gives us.

public:
  ImageProcessor()
    //: it_(nh_)
  {
    // Something is up here!
    // it_sub_ = it_.subscribe("/camera/image", 1,
    //   &ImageProcessor::proc_img, this);
    img_sub_ = nh_.subscribe<>(
      "camera/image/compressed", 1, &ImageProcessor::proc_img, this);
    cmd_pub_ = nh_.advertise<car_serial_comms::ThrottleAndSteering>(
      "vision_controller/drive_cmd", 10);
    cv::namedWindow(WINDOW);
  }

  ~ImageProcessor()
  {
    cv::destroyWindow(WINDOW);
  }

  // proc_img - get new image, move into OpenCV, and process
  void proc_img(const sensor_msgs::CompressedImage& msg) //sensor_msgs::ImageConstPtr&
  {
    // Move image into OpenCV
    // cv_bridge::CvImagePtr im_ptr;
    // try
    // {
    //   im_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }
    cv::Mat img, img_gray, grad;
    img = cv::imdecode(cv::Mat(msg.data),1); // Hax hax hax

    //cv::Mat img = cv::Mat(msg.data); // Just be lazy
    cv::blur(img,img,cv::Size(5,5));

    cv::cvtColor(img,img_gray,CV_RGB2GRAY);

    cv::Mat grad_x, grad_y, grad_x_abs, grad_y_abs;

    // Get gradients
    cv::Sobel(img_gray, grad_x, CV_16S, 1, 0, 3, 1, 0); //, BORDER_DEFAULT);
    cv::Sobel(img_gray, grad_y, CV_16S, 0, 1, 3, 1, 0);

    // Make absolute
    cv::convertScaleAbs(grad_x, grad_x_abs);
    cv::convertScaleAbs(grad_y, grad_y_abs);

    // Add
    cv::addWeighted(grad_x_abs, 0.5, grad_y_abs, 0.5, 0, grad);

    // Display Image
    cv::imshow(WINDOW, grad);
    cv::waitKey(1);

    // Get steering and trottle from image... Somehow...
    // ...

    // Output streering and throttle
    car_serial_comms::ThrottleAndSteering out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "/chassis";
    out_msg.steering = 90;
    out_msg.throttle = 111;

    cmd_pub_.publish(out_msg);
  }
};

/*
 * main - Get images from the Pi, process them, and pass commands to the node
 *        that talks to the Arduino
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "iarrcMlVision");

  // // Advertise the commanded steering and throttle
  // ros::Publisher strThrPub =
  // // Subscribe to publisher from the camera node (only buffer 1 frame, we don't want old ones anyways)
  // ros::Subscriber imageSub = nh.subscribe<sensor_msgs::CompressedImage>("throttle_steering_cmd", 1, image_callback);

  ImageProcessor ip;

  // Let ROS do its thing
  ros::spin(); // Doesn't return unless something goes wrong
  return 0;
}
