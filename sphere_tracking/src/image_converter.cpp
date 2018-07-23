#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>


using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  float min, max;
  float cx;
  float cy;
  float fx;
  float fy;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
  //  image_sub_ = it_.subscribe("/left/rgb/image_rect_color", 1,
  //    &ImageConverter::imageCb, this);
    image_sub_ = it_.subscribe("/left/depth_registered/image_raw", 1, &ImageConverter::imageCb, this);

    image_pub_ = it_.advertise("/image_converter/output_video_depth", 1);

    min=0;
    max=1000000;
    cx = 319.5; // center of projection
    cy = 239.5; // center of projection
    fx = 570.342224121093;//525.0;
    fy = 570.3422241210938;// 525.0;//

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  float dist2Point(float d1, int u1, int v1, float d2, int u2, int v2){
    pcl::PointXYZRGB p1;
    pcl::PointXYZRGB p2;
    float x,y,z;
    float dist;
    /*x = ((u1 - cx)*d1)/fx;
    y = (-1.0*(v1 - cy)*d1)/fy;
    z = (-1.0*d1);*/
    z = d1;
    x = z*2.0*tan(fx/2.0)*(((float)u1-cx)/640.0);
    y = z*2.0*tan(fy/2.0)*(((float)v1-cy)/480.0);
    p1.x = x;
    p1.y = y;
    p1.z = z;
    /*
    x = ((u2 - cx)*d2)/fx;
    y = (-1.0*(v2 - cy)*d2)/fy;
    z = (-1.0*d2);*/
    z = d2;
    x = z*2*tan(fx/2)*((u2-cx)/640);
    y = z*2*tan(fy/2)*((v2-cy)/480);
    p2.x = x;
    p2.y = y;
    p2.z = z;
    dist=sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
    return dist;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //cv::Mat src_gray;

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
    float val=0.0;
    /// Convert it to gray
    cv::Mat src_gray(cv_ptr->image.rows,cv_ptr->image.cols, CV_8UC1);
    for(int i=1;i<cv_ptr->image.rows;i++){
        for(int j=1;j<cv_ptr->image.cols;j++){
            //ROS_INFO("%u ",src_gray.at<unsigned char>(i,j)=((float)cv_ptr->image.at<unsigned short>(i,j))*8000.0/255.0);
            if(((float)cv_ptr->image.at<float>(i,j))<4.0){
                if(((float)cv_ptr->image.at<float>(i,j))>min && ((float)cv_ptr->image.at<float>(i,j))<max){
                    src_gray.at<unsigned char>(i,j)=255;//((float)cv_ptr->image.at<float>(i,j))*255.0/8.0;
                }else{
                    src_gray.at<unsigned char>(i,j)=0;
                }

                if(val<(cv_ptr->image.at<float>(i,j))){
                  val=(cv_ptr->image.at<float>(i,j));
                }
            }else{
                src_gray.at<unsigned char>(i,j)=0;
            }
        }
    }
    //ROS_INFO("%f",val);

  //  cvtColor( src, src_gray, CV_BGR2GRAY );
    ImageConverter::findCircle(src_gray,msg,cv_ptr);
    /// Show your result
    cv::imshow(OPENCV_WINDOW, src_gray);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

  void findCircle(cv::Mat &src_gray, const sensor_msgs::ImageConstPtr& msg,cv_bridge::CvImagePtr cv_ptr){
  /// Reduce the noise so we avoid false circle detection
      GaussianBlur(src_gray, src_gray, cv::Size(9, 9), 2, 2 );
      //blur(src_gray, src_gray, cv::Size(3, 3));

      std::vector<cv::Vec3f> circles;

      /// Apply the Hough Transform to find the circles
      HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 1, 17, 0, 0 );
      /// Draw the circles detected
      int diff;
      float dist;
      /// Data file to save all circle datas+timestamp
      FILE* fichier = NULL;
      int numCircle1, numCircle2;
      /// Save the file in catkin_ws folder
      /// Remove the file or rename it in the folder if you want a new data file.
      fichier = fopen("test_depth.csv", "a+");
      if(circles.size()>=2){ //&& circles.size()<10){
          diff=1000000;
          /// find two circles in the circles array who have the smallest radius difference
          for( size_t i = 0; i < circles.size(); i++ )
          {
              for( size_t j = 0; j < circles.size(); j++ ){
                  if(i!=j){
                      dist=ImageConverter::dist2Point(cv_ptr->image.at<float>(cvRound(circles[i][0]),cvRound(circles[i][1])), cvRound(circles[i][0]), cvRound(circles[i][1]), cv_ptr->image.at<float>(cvRound(circles[j][0]),cvRound(circles[j][1])),  cvRound(circles[j][0]), cvRound(circles[j][1]));
                      //ROS_INFO("dist1 : %f, u1 : %u, v1 : %u, dist2 : %f, u2 : %u, v2 : %u",cv_ptr->image.at<float>(cvRound(circles[i][0]),cvRound(circles[i][1])), cvRound(circles[i][0]), cvRound(circles[i][1]), cv_ptr->image.at<float>(cvRound(circles[j][0]),cvRound(circles[j][1])),  cvRound(circles[j][0]), cvRound(circles[j][1]));
                      //ROS_INFO("%f   ",dist);
                      if(dist>0.5 && dist<0.8 && !std::isnan(dist)){
                        circle( src_gray, cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1])), cvRound(circles[i][2]), cv::Scalar(255,255,255), 3, 8, 0 );
                          if(abs((cvRound(circles[i][2]))-(cvRound(circles[j][2])))<diff){
                              diff=abs((cvRound(circles[i][2]))-(cvRound(circles[j][2])));
                              numCircle1=i;
                              numCircle2=j;
                          }
                      }
                  }
              }
           }
          if(diff<10){
           fprintf(fichier, "%d;%d;", cvRound(circles[numCircle1][0]), cvRound(circles[numCircle1][1]));
           fprintf(fichier, "%d;%d", cvRound(circles[numCircle2][0]), cvRound(circles[numCircle2][1]));
           fprintf(fichier, ";%d", msg->header.stamp.nsec );
           //fprintf(fichier, ";%d", diff );
           fprintf(fichier, "\n");

           fclose(fichier);
           cv::Point center1( cvRound(circles[numCircle1][0]), cvRound(circles[numCircle1][1]));

           int radius = cvRound(circles[numCircle1][2]);
           // circle center
           circle( src_gray, center1, 3, cv::Scalar(0,255,0), -1, 8, 0 );
           // circle outline
           circle( src_gray, center1, radius, cv::Scalar(0,0,255), 3, 8, 0 );

           cv::Point center2( cvRound(circles[numCircle2][0]), cvRound(circles[numCircle2][1]));
           radius = cvRound(circles[numCircle2][2]);
           // circle center
           circle( src_gray, center2, 3, cv::Scalar(0,255,0), -1, 8, 0 );
           // circle outline
           circle( src_gray, center2, radius, cv::Scalar(0,0,255), 3, 8, 0 );
           /// min is the minimum of rang to delete the background for the next frame (I put 30cm arround the sphere)
           //min= (cv_ptr->image.at<float>(cvRound(circles[numCircle1][0]),cvRound(circles[numCircle1][1]))+cv_ptr->image.at<float>(cvRound(circles[numCircle2][0]), cvRound(circles[numCircle2][1]))/2)-1.0;
           //max=(cv_ptr->image.at<float>(cvRound(circles[numCircle1][0]),cvRound(circles[numCircle1][1]))+cv_ptr->image.at<float>(cvRound(circles[numCircle2][0]), cvRound(circles[numCircle2][1]))/2)+1.0;
         }
      }
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
