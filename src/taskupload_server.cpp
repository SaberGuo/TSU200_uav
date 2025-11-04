#include <ros/ros.h>
#include "tsu200/TaskUpload.h"
#include "tsu200/Pos.h"
#include <geometry_msgs/Point.h>

bool handle_taskupload(tsu200::TaskUpload::Request &req, tsu200::TaskUpload::Response &res) {
  ROS_INFO("TaskUpload request id=%d PosNum=%d", req.id, req.PosNum);
  if (req.PosNum != (int)req.PosList.size()) {
    res.id = req.id;
    res.success = false;
    res.status_code = 2;
    res.message = "PosNum does not match PosList length";
    ROS_WARN("%s", res.message.c_str());
    return true;
  }
  // Process positions (simulate)
  for (size_t i = 0; i < req.PosList.size(); ++i) {
    const tsu200::Pos &p = req.PosList[i];
    ROS_INFO("  Pos %lu: x=%f y=%f z=%f type=%s info=%s", i, p.x, p.y, p.z, p.task_type.c_str(), p.info.c_str());
  }
  res.id = req.id;
  res.success = true;
  res.status_code = 0;
  res.message = "Uploaded and scheduled";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "taskupload_server_cpp");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("task_upload", handle_taskupload);
  ROS_INFO("task_upload service ready (C++)");
  ros::spin();
  return 0;
}
