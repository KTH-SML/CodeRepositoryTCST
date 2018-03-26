#include "ros/ros.h"
#include "std_msgs/Bool.h"

void clear_costmap_cb(std_msgs::Bool msg)
{
  ROS_INFO("Got msg");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_service_caller");

  ros::NodeHandle nh;

  ros::Subscriber sub_clear_costmap = nh.subscribe("clear_costmap", 1, clear_costmap_cb);

  ros::spin();

  return 0;

}
