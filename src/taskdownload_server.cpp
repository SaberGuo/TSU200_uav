#include <ros/ros.h>
#include "tsu200/TaskDownload.h"
#include "tsu200/Pos.h"
#include <geometry_msgs/Point.h>

bool handle_taskdownload(tsu200::TaskDownload::Request &req, tsu200::TaskDownload::Response &res) {
  ROS_INFO("TaskDownload request id=%d", req.id);
  res.id = req.id;
  geometry_msgs::Point home;
  home.x = 0.0; home.y = 0.0; home.z = 0.0;
  res.HomePos = home;
  tsu200::Pos p1; p1.x=1.0; p1.y=1.0; p1.z=0.5; p1.task_type="waypoint"; p1.info="first";
  tsu200::Pos p2; p2.x=2.0; p2.y=2.0; p2.z=1.0; p2.task_type="survey"; p2.info="second";
  res.PosList.push_back(p1);
  res.PosList.push_back(p2);
  res.PosNum = res.PosList.size();
  ROS_INFO("Returning %d positions", res.PosNum);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "taskdownload_server_cpp");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("task_download", handle_taskdownload);
  ROS_INFO("task_download service ready (C++)");
  ros::spin();
  return 0;
}
