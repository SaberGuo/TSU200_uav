#include <ros/ros.h>
#include "tsu200/TaskOpt.h"
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "taskopt_client_cpp");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tsu200::TaskOpt>("task_opt");
  tsu200::TaskOpt srv;
  std::string opt = "noop";
  int id = 0;
  if (argc > 1) opt = argv[1];
  if (argc > 2) id = std::atoi(argv[2]);
  srv.request.opt = opt;
  srv.request.id = id;
  if (client.call(srv)) {
    ROS_INFO("success=%s, code=%d, msg=%s", srv.response.success?"true":"false", srv.response.status_code, srv.response.message.c_str());
  } else {
    ROS_ERROR("Failed to call service task_opt");
    return 1;
  }
  return 0;
}
