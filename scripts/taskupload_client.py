#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from tsu200.srv import TaskUpload
from tsu200.msg import Pos


def call_taskupload(id_val, home, positions):
    rospy.wait_for_service('task_upload')
    try:
        svc = rospy.ServiceProxy('task_upload', TaskUpload)
        resp = svc(id_val, home, len(positions), positions)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s', e)
        return None


if __name__ == '__main__':
    rospy.init_node('taskupload_client')
    # Build a default HomePos and positions if none provided
    home = Point(x=0.0, y=0.0, z=0.0)
    pos_list = []
    # Example: create two positions
    p1 = Pos(x=1.0, y=1.0, z=0.0, task_type='takeoff', info='first')
    p2 = Pos(x=2.0, y=2.0, z=1.5, task_type='survey', info='second')
    pos_list.append(p1)
    pos_list.append(p2)

    # Allow overriding id from argv
    id_val = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    resp = call_taskupload(id_val, home, pos_list)
    if resp:
        print('id:', resp.id)
        print('success:', resp.success)
        print('status_code:', resp.status_code)
        print('message:', resp.message)
