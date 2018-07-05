#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <cmath>


using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
static cv::Point2i pt;
//static int var1;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  float min, max;


  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
  //  image_sub_ = it_.subscribe("/left/rgb/image_rect_color", 1,
  //    &ImageConverter::imageCb, this);

    image_sub_ = it_.subscribe("/left/depth_registered/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video_depth", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //cv::Mat src_gray
    cv::Mat src;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

void onMouse(int evt, int x, int y, int flags, void* param) {
  if(evt == CV_EVENT_LBUTTONDOWN) {
      pt.x = x;
      ROS_INFO("%d\n",pt.x);
      pt.y = y;
      ROS_INFO("%d\n",pt.y);
      //var1=0;
  }
}

int main(int argc, char** argv)
{
  pt=cv::Point2i(-1,-1);
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  cv::setMouseCallback(OPENCV_WINDOW, onMouse, 0);
  ros::spin();
  return 0;
}
