#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <DenseSLAMSystem.h>
#include <cv_bridge/cv_bridge.h>
#include<iostream>
#include <sstream>  // for string streams 
#include <string>  // for string
#include <nav_msgs/Odometry.h>


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


using namespace sensor_msgs;
using namespace message_filters;



void callback(const ImageConstPtr& img, const ImageConstPtr& depth, const nav_msgs::OdometryConstPtr& ground_truth)
{

    // save rbg image as png
    cv_bridge::CvImagePtr cv_img_ptr;
    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
      cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
      std::ostringstream str1;
      str1 << "data/";
      str1 << img->header.stamp;
      str1 << ".png";
      std::string name_img = str1.str();
      std::ostringstream str2;
      str2 << "data/";
      str2 << "depth";
      str2 << depth->header.stamp;
      str2 << ".png";
      std::string name_depth = str2.str();
      cv::imwrite(name_img, cv_img_ptr->image);
      cv::imwrite(name_depth, cv_depth_ptr->image);
      ROS_INFO("Img saved %s", name_img);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    } 

  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_sub_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<nav_msgs::Odometry> tf_sub(nh, "/ground_truth/state", 1);
  // message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/rgb/camera_info", 1);
  TimeSynchronizer<Image, Image, nav_msgs::Odometry> sync(rgb_sub, depth_sub, tf_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}