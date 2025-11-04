#!/usr/bin/env python
import rospy
from tsu200.srv import TaskDownload, TaskDownloadResponse
from geometry_msgs.msg import Point
from tsu200.msg import Pos


def handle_taskdownload(req):
    rospy.loginfo("TaskDownload request id=%d", req.id)
    # In a real implementation, look up stored task by id. Here we simulate a response.
    home = Point(x=0.0, y=0.0, z=0.0)

    # Example Pos list (this would normally come from storage or a planner)
    pos_list = []
    pos_list.append(Pos(x=1.0, y=1.0, z=0.5, task_type='waypoint', info='first waypoint'))
    pos_list.append(Pos(x=2.0, y=2.0, z=1.0, task_type='survey', info='second waypoint'))

    pos_num = len(pos_list)
    rospy.loginfo('Returning id=%d HomePos=(%.3f,%.3f,%.3f) PosNum=%d', req.id, home.x, home.y, home.z, pos_num)
    for i, p in enumerate(pos_list):
        rospy.loginfo('  Pos %d: x=%.3f y=%.3f z=%.3f type=%s info=%s', i, p.x, p.y, p.z, p.task_type, p.info)

    return TaskDownloadResponse(req.id, home, pos_num, pos_list)


def taskdownload_server():
    rospy.init_node('taskdownload_server')
    s = rospy.Service('task_download', TaskDownload, handle_taskdownload)
    rospy.loginfo('task_download service ready')
    rospy.spin()


if __name__ == '__main__':
    taskdownload_server()
