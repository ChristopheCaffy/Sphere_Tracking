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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/left/rgb/image_rect_color", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video_RGB", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat src,src_gray;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    /// Convert it to gray
    src=cv_ptr->image;
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 180, 30, 0, 0 );
    /// Draw the circles detected
    int diff;
    /// Data file to save all circle datastimestamp
    FILE* fichier = NULL;
    int numCircle1, numCircle2;
    /// Save the file in catkin_ws folder
    /// Remove the file or rename it in the folder if you want a new data file.
    fichier = fopen("test_RGB.csv", "a+");
    if(circles.size()>=2 && circles.size()<10){
        diff=1000000;
        /// find two circles in the circles array who have the smallest radius difference
        for( size_t i = 0; i < circles.size(); i++ )
        {
            for( size_t j = 0; j < circles.size(); j++ ){
                if(i!=j){
                    if(abs((cvRound(circles[i][2]))-(cvRound(circles[j][2])))<diff){
                        diff=abs((cvRound(circles[i][2]))-(cvRound(circles[j][2])));
                        numCircle1=i;
                        numCircle2=j;
                    }
                }
            }
         }
        if(diff<3){
         fprintf(fichier, "%d;%d;", cvRound(circles[numCircle1][0]), cvRound(circles[numCircle1][1]));
         fprintf(fichier, "%d;%d", cvRound(circles[numCircle2][0]), cvRound(circles[numCircle2][1]));
         fprintf(fichier, ";%d", msg->header.stamp.nsec );
         //fprintf(fichier, ";%d", diff );
         fprintf(fichier, "\n");
         cv::Point center1( cvRound(circles[numCircle1][0]), cvRound(circles[numCircle1][1]));
         int radius = cvRound(circles[numCircle1][2]);
         // circle center
         circle( src, center1, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // circle outline
         circle( src, center1, radius, cv::Scalar(0,0,255), 3, 8, 0 );

         cv::Point center2( cvRound(circles[numCircle2][0]), cvRound(circles[numCircle2][1]));
         radius = cvRound(circles[numCircle2][2]);
         // circle center
         circle( src, center2, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // circle outline
         circle( src, center2, radius, cv::Scalar(0,0,255), 3, 8, 0 );
       }
    }

    fclose(fichier);
    /// Show your result
    cv::imshow(OPENCV_WINDOW, src);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_RGB");
  ImageConverter ic;
  ros::spin();
  return 0;
}
