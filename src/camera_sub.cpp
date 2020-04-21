#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <DenseSLAMSystem.h>

using namespace sensor_msgs;
using namespace message_filters;


void callback(const ImageConstPtr& image, const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info)
{
    std::cout << "I was called" << std::endl;
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_sub_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/rgb/camera_info", 1);
  TimeSynchronizer<Image, Image, CameraInfo> sync(rgb_sub, depth_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}