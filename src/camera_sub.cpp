#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "tf/transform_datatypes.h"
#include <eigen_conversions/eigen_msg.h>


#include <cv_bridge/cv_bridge.h>
#include<iostream>
#include <sstream>  // for string streams 
#include <string>  // for string
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


using namespace sensor_msgs;
using namespace message_filters;



int counter = 0;

std::string getFilename(std::string ext){
  std::string front = "scene_00_";
  std::string number;
  if (counter < 10){
    number = "000";
  }
  else if(counter < 100){
    number = "00";
  }
  else{
    number = "0";
  }
  std::string counter_str = std::to_string(counter);

  return front + number + counter_str + ext;

}

void writeMatToFile(cv::Mat& m, const char* filename)
{
    std::ofstream fout(filename);

    if(!fout)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout<<m.at<float>(i,j)<<"\t";
        }
    }

    fout.close();
}

void saveRgb(const ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_img_ptr;
    cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    std::string fileName = getFilename(".png");
    try {
    cv::imwrite(fileName, cv_img_ptr->image);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


void saveDepth(const ImageConstPtr& depth)
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    std::string fileName = getFilename(".depth");
    cv::Mat image_mat = cv_depth_ptr->image;
    cv::patchNaNs(image_mat, 100000);
    writeMatToFile(image_mat, fileName.c_str());

}

void savePose(const nav_msgs::OdometryConstPtr& msg)
{
  std::string fileName = "gt_pose.txt";
  std::string fileNameEuler = "gt_pose_euler.txt";

  Eigen::Vector3f zero_pos (10.0, 20.0, 0.00);

  Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_WB;
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, q_WB);
  T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
  Eigen::Vector3d t_WB;
  tf::pointMsgToEigen(msg->pose.pose.position, t_WB);
  T_WB.topRightCorner<3, 1>() = t_WB;


  Eigen::Quaterniond quat_world_body = q_WB;


  Eigen::Vector3f tran ((float) msg->pose.pose.position.x,
                        (float) msg->pose.pose.position.y,
                        (float) msg->pose.pose.position.z);

  float x_t = 0.069, y_t = -0.047, z_t =  0.117;
  Eigen::Vector3f footprint_camera_pose = Eigen::Vector3f(x_t, y_t, z_t); // rgb camera route
  Eigen::Quaterniond quat_body_camera = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5);
  
  Eigen::Vector3f res_tran = (tran - zero_pos) + footprint_camera_pose;
  Eigen::Quaterniond res_quat = quat_world_body * quat_body_camera; 


  res_quat.normalize(); // normalize quaternion after operations
  
  // rotation_matrix
  Eigen::Matrix3d rot_matrix = res_quat.toRotationMatrix();

  std::ofstream writer(fileName, std::ios_base::app | std::ios_base::out);
  writer << counter << " "; // frame number as the first number
  writer << res_tran[0] << " " << res_tran[1] << " " << res_tran[2] << " ";
  writer << res_quat.x() << " " << res_quat.y() << " " << res_quat.z() << " "
         << res_quat.w()<< std::endl;
  writer.close();


  std::ofstream writerEuler(fileNameEuler, std::ios_base::app | std::ios_base::out);
  writerEuler << rot_matrix<< std::endl;
  writerEuler.close();

}


void callback(const ImageConstPtr& img, const ImageConstPtr& depth, const nav_msgs::OdometryConstPtr& ground_truth) {


  saveRgb(img);
  saveDepth(depth);
  savePose(ground_truth);
  ROS_INFO("Img saved");

  counter++;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_sub_node");

  ros::NodeHandle nh;


  message_filters::Subscriber<Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<nav_msgs::Odometry> tf_sub(nh, "/ground_truth/state", 1);
  TimeSynchronizer<Image, Image, nav_msgs::Odometry> sync(rgb_sub, depth_sub, tf_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}
