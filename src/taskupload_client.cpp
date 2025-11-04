#include <ros/ros.h>
#include "tsu200/TaskUpload.h"
#include "tsu200/Pos.h"
#include <geometry_msgs/Point.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "taskupload_client_cpp");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tsu200::TaskUpload>("task_upload");
  tsu200::TaskUpload srv;

  int id = 1;
  if (argc > 1) id = atoi(argv[1]);

  geometry_msgs::Point home;
  home.x = 0.0; home.y = 0.0; home.z = 0.0;
  srv.request.id = id;
  srv.request.HomePos = home;

  tsu200::Pos p1; p1.x = 1.0; p1.y = 1.0; p1.z = 0.0; p1.task_type = "takeoff"; p1.info = "first";
  tsu200::Pos p2; p2.x = 2.0; p2.y = 2.0; p2.z = 1.5; p2.task_type = "survey"; p2.info = "second";
  srv.request.PosList.push_back(p1);
  srv.request.PosList.push_back(p2);
  srv.request.PosNum = srv.request.PosList.size();

  if (client.call(srv)) {
    ROS_INFO("id=%d success=%s status_code=%d message=%s", srv.response.id, srv.response.success?"true":"false", srv.response.status_code, srv.response.message.c_str());
  } else {
    ROS_ERROR("Failed to call service task_upload");
    return 1;
  }
  return 0;
}
