#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

static const std::string OPENCV_WINDOW = "Image window";

class RGBplusDEPTH{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cvPtr_, cv_ptr_f_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels

    struct DataBuffer
    {
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex buffer_mutex_;
        cv_bridge::CvImagePtr cv_ptr;
    };
    DataBuffer bufferRGB;//! And now we have our container

    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex buffer_mutex_;
    };
    ImageDataBuffer bufferDepth;//! And now we have our container


public:
    RGBplusDEPTH(ros::NodeHandle nh)
    : it_(nh)
    {
        image_sub_ = it_.subscribe("/left/rgb/image_rect_color", 1, &RGBplusDEPTH::imageCallback, this);
        depth_sub_ = it_.subscribe("left/depth_registered/image_raw", 1, &RGBplusDEPTH::depthCallback, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
    }

    ~RGBplusDEPTH()
    {
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      //! Below code pushes the image and time to a deque, to share across threads
        try
        {
            cvPtr_ = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        bufferDepth.buffer_mutex_.lock();
        bufferDepth.imageDeq.push_back(cvPtr_->image);
        bufferDepth.timeStampDeq.push_back(msg->header.stamp);
        if(bufferDepth.imageDeq.size()>2){
            bufferDepth.imageDeq.pop_front();
            bufferDepth.timeStampDeq.pop_front();
        }
        bufferDepth.buffer_mutex_.unlock();

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        //! Below code pushes the image and time to a deque, to share across threads
        try
        {
          if (sensor_msgs::image_encodings::isColor(msg->encoding)){
            cvPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            bufferRGB.cv_ptr=cvPtr_;
          }
          else{
            cvPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            bufferRGB.cv_ptr=cvPtr_;
          }
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        bufferRGB.buffer_mutex_.lock();
        bufferRGB.imageDeq.push_back(cvPtr_->image);
        bufferRGB.timeStampDeq.push_back(msg->header.stamp);
        if(bufferRGB.imageDeq.size()>2){
            bufferRGB.imageDeq.pop_front();
            bufferRGB.timeStampDeq.pop_front();
        }
        bufferRGB.buffer_mutex_.unlock();

    }


    void seperateThread() {
       /**
        * The below loop runs until ros is shutdown, to ensure this thread does not remain
        * a zombie thread
        *
        * The loop locks the buffer, checks the size
        * And then pulls items: the pose and timer_t
        * You can contemplate weather these could have been combined into one ...
        */

        /// The below gets the current Ros Time
        ros::Time timeImageDepth = ros::Time::now();;
        ros::Time timeImageRGB = ros::Time::now();;
        cv::Mat imageRGB;
        cv::Mat mat_depth;
        cv::Mat src,src_gray;

        ros::Rate rate_limiter(1.0);
        while (ros::ok()) {

            bufferRGB.buffer_mutex_.lock();
            if(bufferRGB.imageDeq.size()>0){
                cv_ptr_f_= bufferRGB.cv_ptr;
                imageRGB = bufferRGB.imageDeq.front();
                timeImageRGB = bufferRGB.timeStampDeq.front();
                bufferRGB.imageDeq.pop_front();
                bufferRGB.timeStampDeq.pop_front();
            }
            bufferRGB.buffer_mutex_.unlock();

            bufferDepth.buffer_mutex_.lock();
            if(bufferDepth.imageDeq.size()>0){
                mat_depth = bufferDepth.imageDeq.front();
                timeImageDepth = bufferDepth.timeStampDeq.front();
                bufferDepth.imageDeq.pop_front();
                bufferDepth.timeStampDeq.pop_front();
            }
            bufferDepth.buffer_mutex_.unlock();

            // convert in a image CV_8UC1 pour voir le depth correctement et pouvoir faire des traitement dessus et utiliser rqt_bag
            // diviser par 8000 et multiplier par 255 pour avoir le bon scale
            //unsigned short Di
            // dt = 1/30s
            //

            /*const int mySizes[2]={mat_depth.rows,mat_depth.cols};
            cv::Mat imageDepth = cv::Mat::zeros(2,mySizes,CV_16UC1);
            //cv::Mat imageDepth = cv::Mat::zeros(mat_depth.rows,mat_depth.cols, CV_16UC1);
          /*  if(!mat_depth.empty()){
                for(int i=1;i<mat_depth.rows;i++){
                    for(int j=1;j<mat_depth.cols;j++){
                        // imageDepth.at<int>(i,j)=*(mat_depth.data+mat_depth.step[0]*i+mat_depth.step[1]*j);
                        imageDepth.at<int>(i,j)=mat_depth.at<int>(i,j);
                         //ROS_INFO("valeur trouvée");
                    }
                }
                //ROS_INFO("valeur trouvée");
            }
*/
            //image processing to detect circles in the frame
            if(!imageRGB.empty()){
                /// Convert it to gray
                src=imageRGB;
                cv::cvtColor( src, src_gray, CV_BGR2GRAY );

                /// Reduce the noise so we avoid false circle detection
                GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

                std::vector<cv::Vec3f> circles;

                /// Apply the Hough Transform to find the circles
                HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 180, 30, 0, 0 );

                /// Draw the circles detected

                for( size_t i = 0; i < circles.size(); i++ )
                {
                    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                    int radius = cvRound(circles[i][2]);
                    // circle center
                    circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                    // circle outline
                    circle( src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

                 }

                /// Show your result
                cv::imshow(OPENCV_WINDOW, src);
                cv::waitKey(3);

                // Output modified video stream
                image_pub_.publish(cv_ptr_f_->toImageMsg());
            }
            rate_limiter.sleep();
        }
    }

};


int main(int argc, char **argv)
{


    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "main_thread");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  std::shared_ptr<RGBplusDEPTH> gc(new RGBplusDEPTH(nh));
  std::thread t(&RGBplusDEPTH::seperateThread,gc);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  t.join();

  return 0;
}
