// ROS includes
#include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Get message prototype for sending to the Arduino managing node
#include <car_serial_comms/ThrottleAndSteering.h>

// Get code copied from the OpenCV cookbook
#include "iarrcMlVision/linefinder.h"

// Make sure that this is not defined on the Raspberry Pi!
//#define DISPLAY

#ifdef DISPLAY
// Name for OpenCV's display window (used for debugging only)
static const std::string WINDOW = "Subscribed Image";
#endif

/*
 * class ImageProcessor
 *
 * Handles (attempts) line detection and getting a steering angle and throttle
 * from images provided by the Raspberry Pi camera node. Note that since that
 * node is kinda broken, this will not work with other video feeds!!!
 */
class ImageProcessor
{
  // Member variables
  ros::NodeHandle nh_;
  // image_transport::ImageTransport it_;
  // image_transport::Subscriber it_sub_;

  ros::Publisher cmd_pub_;
  ros::Subscriber img_sub_; // This is very bad practice, but we're stuck
                              // with what the raspicam_node gives us.

  int houghVote_;

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

    // For regular Hough transform
    houghVote_ = -1; // Force to be reset

    // Comment when compiling on the Pi
    #ifdef DISPLAY
    cv::namedWindow(WINDOW);
    #endif
  }

  ~ImageProcessor()
  {
    #ifdef DISPLAY
    cv::destroyWindow(WINDOW);
    #endif
  }

  // proc_img - get new image, move into OpenCV, and process
  void proc_img(const sensor_msgs::CompressedImage& msg) //sensor_msgs::ImageConstPtr& (proper way)
  {
    // Move image into OpenCV - the proper way that I refuse to delete
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

    // Hax hax hax
    cv::Mat img;
    img = cv::imdecode(cv::Mat(msg.data),1);
    // Hax hax hax

    //==========================================================================
    // Line detector (heavilly borrowed from the internet)
    // see www.transistor.io/revisiting-lane-detection-using-opencv.html
    // Many thanks to the author for making his code available.

    // TODO Make this function work

    // TODO Add a ROI (just grab a subset of the image)

    // Canny edge detection
    cv::Mat contours;
    // TUNE Make sure these parameters are good for various conditions
    cv::Canny(img, contours, 50, 350);
    #ifdef DISPLAY
    cv::Mat contoursInv;
    cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);
    #endif

    // // Hough transform
    // // Note: houghVote_ is the min number of points must be found to be a line
    // // Not as good as the probabalistic hough transform aparently!
    // std::vector<Vec2f> lines;
    // if (houghVote_ < 1)
    // { // we lost all lines. reset // or lines.size() > 2
    //     houghVote_ = 200;
    // }
    // else
    // {
    //   houghVote_ += 10; // Increment so we don't miss lines
    // }
    // // Do first transform
    // HoughLines(contours, lines, 1, PI/180, houghVote_);
    // // Mess with vote iff results were bad
    // while(lines.size() > 5 && houghVote__ > 0) // reduce until 4 lines
    // {
    //   houghVote_ -= 5;
    //   HoughLines(contours, lines, 1, PI/180, houghVote_);
    // }
    // ROS_INFO("houghVote_ = %d", houghVote_);
    // Mat result(contours.rows,contours.cols,CV_8U,Scalar(255));
    // image.copyTo(result); // overwrite??? Confused!

    // Probabalistic Hough transform (better)
    LineFinder lf; // From OpenCV cookbook, see included linefinder.h
    lf.setLineLengthAndGap(60, 10); // min len (pix), min gap (pix)
    lf.setMinVote(4); // minimum number of points to be a line
    std::vector<cv::Vec4i> lines = lf.findLines(contours); // TODO check if [x1, y1, x2, y2] (use OpenCV's docs)
    #ifdef DISPLAY
    cv::Mat houghP(img.size(), CV_8U, cv::Scalar(0));
    lf.drawDetectedLines(houghP);
    #endif
    // TODO
    // Grab the two longest lines and use their angles and positions to control
    // the steering.  Maybe use length for speed?  Long ==> straight-away?


    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // // Simple image gradient calculation - pretty useless
    // cv::Mat img_gray, grad;
    // cv::blur(img,img,cv::Size(5,5));
    //
    // cv::cvtColor(img,img_gray,CV_RGB2GRAY);
    //
    // cv::Mat grad_x, grad_y, grad_x_abs, grad_y_abs;
    //
    // // Get gradients
    // cv::Sobel(img_gray, grad_x, CV_16S, 1, 0, 3, 1, 0); //, BORDER_DEFAULT);
    // cv::Sobel(img_gray, grad_y, CV_16S, 0, 1, 3, 1, 0);
    //
    // // Make absolute
    // cv::convertScaleAbs(grad_x, grad_x_abs);
    // cv::convertScaleAbs(grad_y, grad_y_abs);
    //
    // // Add
    // cv::addWeighted(grad_x_abs, 0.5, grad_y_abs, 0.5, 0, grad);
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Display Image
    #ifdef DISPLAY
    cv::imshow(WINDOW, houghP);
    cv::waitKey(1);
    #endif

    // TODO
    // Get steering and trottle from image... Somehow...
    // ...

    // End of line detector
    //==========================================================================

    //**********************************************
    // Output streering and throttle
    car_serial_comms::ThrottleAndSteering out_msg; // Message to send
    out_msg.header.stamp = ros::Time::now(); // Record the time
    out_msg.header.frame_id = "/chassis"; // Just for fun
    out_msg.steering = 0; // Steering angle, -90 to +90 (ask Kevin)
    out_msg.throttle = 9; // Speed in m/s
    cmd_pub_.publish(out_msg); // Send it
    //**********************************************
  }
};

/*
 * main - Get images from the Pi, process them, and pass commands to the node
 *        that talks to the Arduino
 */
int main(int argc, char** argv)
{
  // Let ROS handle its setup stuff
  ros::init(argc, argv, "iarrcMlVision");

  // Create the image processor. Uses callbacks to grab each new frame.
  ImageProcessor ip;

  // Let ROS do its thing
  ros::spin(); // Doesn't return unless we need to close

  // Close
  return 0;
}
