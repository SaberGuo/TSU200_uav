#include <ros/ros.h>
#include "tsu200/TaskOpt.h"

bool handle_taskopt(tsu200::TaskOpt::Request &req, tsu200::TaskOpt::Response &res) {
  ROS_INFO("TaskOpt request: opt='%s', id=%d", req.opt.c_str(), req.id);
  res.success = true;
  res.status_code = 0;
  res.message = "Executed: " + req.opt;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "taskopt_server_cpp");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("task_opt", handle_taskopt);
  ROS_INFO("task_opt service ready (C++)");
  ros::spin();
  return 0;
}
