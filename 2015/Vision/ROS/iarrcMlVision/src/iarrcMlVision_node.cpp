// ROS includes
#include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/console.h> //DEBUG

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Get message prototype for sending to the Arduino managing node
#include <car_serial_comms/ThrottleAndSteering.h>

// Get code copied from the OpenCV cookbook
#include "iarrcMlVision/linefinder.h"

// STL includes
#include <cmath>
#include <string>

// Make sure that this is not defined on the Raspberry Pi!
//#define DISPLAY


// Utility Functions
template <typename T>
std::string NumberToString ( T Number )
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

double RAD_TO_DEG (double x)
{
  return x * 180 / PI;
}

double DEG_TO_RAD (double x)
{
  return x * PI / 180;
}




// Image utility functions
// This function calculates the distance between two points
double distance (double x1, double y1, double x2, double y2)
{
  return sqrt(abs(pow(y2-y1, 2) + pow (x2-x1, 2)));
}
//OpenCV-friendly overload!
double distance (cv::Point p1, cv::Point p2)
{
  return distance (p1.x, p1.y, p2.x, p2.y);
}

// Finds the intersection point between two lines
cv::Point2f computeIntersect(cv::Vec4i t1, cv::Vec4i t2)
{
  int x1 = t1[0], y1 = t1[1], x2 = t1[2], y2 = t1[3], x3 = t2[0], y3 = t2[1], x4 = t2[2], y4 = t2[3];
  double d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
  if (d == 0) return cv::Point2f(0,0);
  
  double xi = ((x3-x4)*(x1*y2-y1*x2)-(x1-x2)*(x3*y4-y3*x4))/d;
  double yi = ((y3-y4)*(x1*y2-y1*x2)-(y1-y2)*(x3*y4-y3*x4))/d;
  
  // If this point actually intersects with the drawn lines, and not the drawn lines' extension
  if ((xi >= x3 && xi <= x4 || xi >= x4 && xi <= x3) && (yi >= y3 && yi <= y4 || yi >= y4 && yi <= y3) && (xi >= x1 && xi <= x2 || xi >= x2 && xi <= x1) && (yi >= y1 && yi <= y2 || yi >= y2 && yi <= y1))
    return cv::Point2f(xi,yi);
  else
    return cv::Point2f(0,0);
}

//OpenCV goes clockwise from a vector pointing right
int angleOfLine (cv::Vec4i closestLine)
{
  double dx = closestLine[2] - closestLine[0];
  double dy = -(closestLine[3] - closestLine[1]); //negative to avoid the upside-downness of openCV's image; it's confusing me
  
  return RAD_TO_DEG(atan(dx/dy));
}

cv::Vec4i formVectors(int startx, int starty, double vecAngle, double vecMagnitude)
{
  return cv::Vec4i(startx, starty, startx + vecMagnitude * cos(DEG_TO_RAD(vecAngle)), starty + vecMagnitude * sin(DEG_TO_RAD(vecAngle)));
}

cv::Point formEndPoint(cv::Point startPoint, double angle, double magnitude)
{
  return cv::Point(startPoint.x + magnitude * cos(DEG_TO_RAD(angle)), startPoint.y + magnitude * sin(DEG_TO_RAD(angle)));
}


class probeVectors
{
private:
  cv::Point startPos;
  cv::Point endPos;
  double angle;
  double magnitude;
  
  cv::Point textPos;
  int textSize;
  cv::Scalar textColour;
  
  cv::Vec4i cvVector; // This holds the vector that represents the car's direction in CV's visual world
  cv::Vec4i closestLine; // This holds the closest line that collided with this line
  cv::Point intersectionPoint;
  double distToCollision;
  bool collisionFound;
  

public:
  static const int carAngle = -30; // Specifies the angle of direction of the car relative to the footage
  static const int warpCombat = 15; // HAX to combat fisheye until it's dewarped
  
  probeVectors(int x, int y, double ang, double mag, int tx, int ty, int ts = 3, cv::Scalar tc = cv::Scalar(0, 255, 0)) : textPos(tx, ty),
    startPos(x,y), textColour (tc)
  {
    cvVector = formVectors (x, y, ang + 270, mag);
    endPos.x = cvVector[2];
    endPos.y = cvVector[3];
    angle = ang;
    magnitude = mag;
    
    textSize = ts;
    
    distToCollision = 500000; // a very large arbitrary value that will end up overwritten
    collisionFound = false;
  }
  const int& operator[] (int x) const
  {
    return cvVector[x];
  }
  
  operator const cv::Vec4i&() const
  {
    return cvVector;
  }
  
  // Checks if this line collides with any of the given lines, returns true and saves the closest colliding line if so, else returns false
  bool checkForClosestCollision (std::vector<cv::Vec4i> lines)
  {
    // Zero out the old value in closestLine
    for (int i = 0; i < 4; i++)
      closestLine[i] = 0;
    
    
    distToCollision = 500000; // a very large arbitrary value that will end up overwritten
    
    for (unsigned int i = 0;i<lines.size();i++)
    {
      cv::Point2f pt = computeIntersect(cvVector,lines[i]); // Find the intersection of this line and the car's line, if it exists
      if (pt.x >= 0 && pt.y >=0 && pt.y < startPos.y) //If this line is in a valid position in front of the car (last term possibly redundant due to changes in computeIntersect?)
      {
        double adist = distance (pt, startPos);
        if ( adist < distToCollision && adist <= magnitude) // if this line is closer than a previously-found one (last term possibly redundant due to changes in computeIntersect?)
        {
          distToCollision = adist;
          intersectionPoint = pt;
          closestLine = lines[i]; // record this line
        }
      }
    }
    
    if (distToCollision < 500000)
    {
      collisionFound = true;
      return collisionFound;
    }
    else
    {
      collisionFound = false;
      return collisionFound;
    }
  }
  
  // The new angle must be the angle of the line minus the angle of the car to get angle of the new direction relative to car, not y-axis
  int getAngle() const
  {
    if (collisionFound == false)
      return 0;
    
    int safeAngle = angleOfLine(closestLine) - angle;
    
    safeAngle -= safeAngle/abs(safeAngle) * warpCombat; // Hax to try to combat the fisheye
    
    /*if (safeAngle > 90)
      safeAngle = 90;
    else if (safeAngle < -90)
      safeAngle = -90;*/
    
    return safeAngle;
  }
  
  void overlayData (cv::Mat& colourImg) const
  {
    int safeAngle = getAngle();
    int displayAngle = safeAngle + 270 + angle; //add 270 + CAR_ANGLE so that the angle is relative to OpenCV's axis
    
    cv::line(colourImg, startPos, endPos, cv::Scalar(0, 0, 255), 3); // Display car direction
    cv::line(colourImg, startPos, (collisionFound) ? intersectionPoint : startPos, cv::Scalar(255, 0, 0), 3); // Display line to collision
    cv::line(colourImg, startPos, (collisionFound) ? formEndPoint(startPos, displayAngle, magnitude) : startPos, cv::Scalar(0, 255, 0), 3); // Display car plotted direction
    
    putText(colourImg, NumberToString(safeAngle).c_str(), textPos, cv::FONT_HERSHEY_PLAIN, textSize, textColour); // Display angle being sent to Kevin
  }
  
  static int getConsensusAngle(std::vector<probeVectors>* instances, std::vector<cv::Vec4i>& lines)
  {
    int counter(0), sum(0);
    
    for (int iter = 0; iter < instances->size(); iter++)
    {
      if (instances->at(iter).checkForClosestCollision(lines) == true)
      {
        int inputAngle = instances->at(iter).getAngle() + instances->at(iter).angle - carAngle;
        sum += inputAngle; // Find the angle relative to the car's direction of motion
        ++counter;
      }
    }
    
    int safeAngle = (counter != 0) ? sum/counter : 0;
    
    if (safeAngle > 90)
     safeAngle = 90;
     else if (safeAngle < -90)
     safeAngle = -90;
    
    return safeAngle;
  }
};


// Trackbar code (got too lazy to bother figuring out how to stuff both nh_ object and the parameter string into userdata... so copy-paste)
void canny_1Trackbar(int trackbarValue, void* ud)
{
  ros::NodeHandle* userdata = (ros::NodeHandle*) ud;
  userdata->setParam("iarrcMlVision/canny_1", trackbarValue);
}
void canny_2Trackbar(int trackbarValue, void* ud)
{
  ros::NodeHandle* userdata = (ros::NodeHandle*) ud;
  userdata->setParam("iarrcMlVision/canny_2", trackbarValue);
}
void min_lenTrackbar(int trackbarValue, void* ud)
{
  ros::NodeHandle* userdata = (ros::NodeHandle*) ud;
  userdata->setParam("iarrcMlVision/min_len", trackbarValue);
}
void min_gapTrackbar(int trackbarValue, void* ud)
{
  ros::NodeHandle* userdata = (ros::NodeHandle*) ud;
  userdata->setParam("iarrcMlVision/min_gap", trackbarValue);
}
void min_vteTrackbar(int trackbarValue, void* ud)
{
  ros::NodeHandle* userdata = (ros::NodeHandle*) ud;
  userdata->setParam("iarrcMlVision/min_vte", trackbarValue);
}

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

  std::vector <probeVectors> probes; // Holds the probes

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
  }

  ~ImageProcessor()
  {
    #ifdef DISPLAY
    cv::destroyAllWindows();
    #endif
  }

  // white_filter - boost white and remove non-white features
  // Give a uint8 colour image - I don't know how it will behave otherwise!
  void white_filter(cv::Mat &img)
  {
    // cv::Mat dst32;
    // img.convertTo(dst32, CV_32FC3);
    // cv::Mat spl[3];
    // split(dst32,spl);
    // spl[0] = spl[0].mul(spl[1]);
    // spl[0] = spl[0].mul(spl[2]);
    // // Thresholding works poorly...
    // //threshold(spl[0], dst32, 200*200*200, 255, 0);
    // //dst32.convertTo(img, CV_8UC1);
    // cv::normalize(spl[0],img,0,255,CV_MINMAX,CV_8UC1);

    // Try HSV conversion instead
    // cv::Mat dst;
    // cv::cvtColor(img, dst, CV_BGR2HSV);
    int max_sat;
    nh_.param("iarrcMlVision/max_sat",max_sat, 150); // Rejects yellow lines
    // cv::inRange(dst,cv::Scalar(0,0,20),cv::Scalar(255,max_sat,255),img);
    cv::inRange(img,cv::Scalar(max_sat,max_sat,max_sat),cv::Scalar(255,255,255),img);
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

    if (probes.empty())
    {
      //Intialize probes
      probes.push_back(probeVectors(img.cols * .25, img.rows * 0.8, 0, (double)175 * sqrt(pow((double)img.cols/640, 2)+ pow((double)img.rows/480, 2)) / sqrt(2), img.cols *.5, 50, 5, cv::Scalar(0, 255, 0)));
      probes.push_back(probeVectors(img.cols * .25, img.rows * 0.8, -30, (double)150 * sqrt(pow((double)img.cols/640, 2) + pow((double)img.rows/480, 2)) / sqrt(2), 10, 50, 5, cv::Scalar(255, 255, 0)));
      probes.push_back(probeVectors(img.cols * .25, img.rows * 0.8, 45, (double)225 * sqrt(pow((double)img.cols/640, 2) + pow((double)img.rows/480, 2)) / sqrt(2), img.cols *.75, 50, 5, cv::Scalar(255,0,100)));
    }
    
    // Display Subscribed Image
    #ifdef DISPLAY
    cv::imshow("Subscribed Image", img);
    #endif

    // // Convert to 'white-scale'
    // white_filter(img);
    // #ifdef DISPLAY
    // cv::imshow("White-scaled Image", img);
    // #endif

    //==========================================================================
    // Line detector (heavilly borrowed from the internet)
    // see www.transistor.io/revisiting-lane-detection-using-opencv.html
    // Many thanks to the author for making his code available.

    // TODO Make this function work

    // TODO Add a ROI (just grab a subset of the image)

    // Canny edge detection
    cv::Mat contours;
    // TUNE Make sure these parameters are good for various conditions
    int a,b;
    nh_.param("iarrcMlVision/canny_1",a, 85); // These both make the transform reject more.
    nh_.param("iarrcMlVision/canny_2",b,380); // Originally 50, 350
    cv::Canny(img, contours, a, b);
    #ifdef DISPLAY
    //cv::Mat contoursInv;
    //cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);
    cv::createTrackbar("Canny Lower", "Canny Transformed Image", &a, 600, canny_1Trackbar, &nh_);
    cv::createTrackbar("Canny Upper", "Canny Transformed Image", &b, 600, canny_2Trackbar, &nh_);
    cv::imshow("Canny Transformed Image",  contours);
    #endif

    // Black out edges that are parts of the car by just drawing over them
    cv::rectangle(contours,cv::Point(0,contours.rows),cv::Point((int)contours.cols*5/8,(int)contours.rows*3/4),cv::Scalar(0),-1);

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
    int min_len, min_gap, min_vte;
    nh_.param("iarrcMlVision/min_len", min_len, 60); // Originally 60
    nh_.param("iarrcMlVision/min_gap", min_gap, 15); // Originally 10
    nh_.param("iarrcMlVision/min_vte", min_vte, 15); // Originally  4
    lf.setLineLengthAndGap(min_len, min_gap); // min len (pix), min gap (pix)
    lf.setMinVote(min_vte);               // minimum number of points to be a line
    std::vector<cv::Vec4i> lines = lf.findLines(contours); // TODO check if [x1, y1, x2, y2] (use OpenCV's docs)
    cv::Mat houghP(img.size(), CV_8U, cv::Scalar(0));
    lf.drawDetectedLines(houghP);
    #ifdef DISPLAY
    cv::createTrackbar("Min Length", "P Hough Transformed Image", &min_len, 200, min_lenTrackbar, &nh_);
    cv::createTrackbar("Min Gap", "P Hough Transformed Image", &min_gap, 50, min_gapTrackbar, &nh_);
    cv::createTrackbar("Min Vote", "P Hough Transformed Image", &min_vte, 30, min_vteTrackbar, &nh_);
    cv::imshow("P Hough Transformed Image", houghP);
    #endif
    // TODO
    // Grab the two longest lines and use their angles and positions to control
    // the steering.  Maybe use length for speed?  Long ==> straight-away?

    // Distance transform
    // Invert image
    /*cv::threshold(houghP,houghP,128,255,cv::THRESH_BINARY_INV);
    cv::Mat dst32;
    cv::distanceTransform(houghP,dst32,CV_DIST_L2,3);
    #ifdef DISPLAY
    cv::Mat dstDisp;
    cv::normalize(dst32,dstDisp,0.0,1.0,CV_MINMAX);
    cv::imshow("Distance Transoformed Image", dstDisp);
    #endif
     */
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

    // Find intersections with the car direction, and returns the safest angle relative to the car in which to proceed
    
    int angle = probeVectors::getConsensusAngle(&probes, lines); // Holds angle to send to Kevin
    
    /*if (car_direction.checkForClosestCollision(lines))
      angle = car_direction.getAngle();
    
    if (testProbe.checkForClosestCollision(lines))
      testProbe.getAngle();
    
    if (test2Probe.checkForClosestCollision(lines))
      test2Probe.getAngle();*/
    
    
    // Display Image
    #ifdef DISPLAY
    cv::Mat colourOverlay;
    cvtColor(houghP, colourOverlay, CV_GRAY2RGB);
    cv::subtract(cv::Scalar::all(255),colourOverlay, colourOverlay);
    for (int i = 0; i < probes.size(); i++)
      probes[i].overlayData(colourOverlay);
    putText(colourOverlay, NumberToString(angle).c_str(), cv::Point(img.cols / 2, 100), cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar (100,255,255));
    
    cv::imshow("P Hough Transformed Image", colourOverlay);
 
    cv::waitKey(1); // Give OpenCV a chance to draw the images
    #endif

    // End of line detector
    //==========================================================================

    //**********************************************
    // Output streering and throttle
    car_serial_comms::ThrottleAndSteering out_msg; // Message to send
    out_msg.header.stamp = ros::Time::now(); // Record the time
    out_msg.header.frame_id = "/chassis"; // Just for fun
    out_msg.steering = angle; // Steering angle, -90 to +90 (ask Kevin)
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
