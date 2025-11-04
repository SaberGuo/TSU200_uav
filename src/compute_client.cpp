#include <ros/ros.h>
#include "tsu200/Compute.h"
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_client_cpp");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tsu200::Compute>("compute");
  tsu200::Compute srv;
  int a = 1, b = 2;
  if (argc > 2) {
    a = std::atoi(argv[1]);
    b = std::atoi(argv[2]);
  }
  srv.request.a = a;
  srv.request.b = b;
  if (client.call(srv)) {
    ROS_INFO("Result: %d", srv.response.sum);
  } else {
    ROS_ERROR("Failed to call service compute");
    return 1;
  }
  return 0;
}
