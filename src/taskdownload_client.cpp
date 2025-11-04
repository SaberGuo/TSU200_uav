#include <ros/ros.h>
#include "tsu200/TaskDownload.h"
#include "tsu200/Pos.h"
#include <geometry_msgs/Point.h>
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "taskdownload_client_cpp");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tsu200::TaskDownload>("task_download");
  tsu200::TaskDownload srv;
  int id = 1;
  if (argc > 1) id = atoi(argv[1]);
  srv.request.id = id;
  if (client.call(srv)) {
    ROS_INFO("id=%d HomePos=(%f,%f,%f) PosNum=%d", srv.response.id, srv.response.HomePos.x, srv.response.HomePos.y, srv.response.HomePos.z, srv.response.PosNum);
    for (size_t i=0;i<srv.response.PosList.size();++i) {
      const tsu200::Pos &p = srv.response.PosList[i];
      ROS_INFO("  Pos %lu: x=%f y=%f z=%f type=%s info=%s", i, p.x, p.y, p.z, p.task_type.c_str(), p.info.c_str());
    }
  } else {
    ROS_ERROR("Failed to call service task_download");
    return 1;
  }
  return 0;
}
