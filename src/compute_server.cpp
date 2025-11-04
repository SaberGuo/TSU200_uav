#include <ros/ros.h>
#include "tsu200/Compute.h"

bool handle_compute(tsu200::Compute::Request &req, tsu200::Compute::Response &res) {
  ROS_INFO("Compute request: %d + %d", req.a, req.b);
  res.sum = req.a + req.b;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_server_cpp");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("compute", handle_compute);
  ROS_INFO("Ready to compute (C++)");
  ros::spin();
  return 0;
}
