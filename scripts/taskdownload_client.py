#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from tsu200.srv import TaskDownload
from tsu200.msg import Pos


def call_taskdownload(id_val):
    rospy.wait_for_service('task_download')
    try:
        svc = rospy.ServiceProxy('task_download', TaskDownload)
        resp = svc(id_val)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s', e)
        return None


if __name__ == '__main__':
    rospy.init_node('taskdownload_client')
    id_val = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    resp = call_taskdownload(id_val)
    if resp:
        print('id:', resp.id)
        print('HomePos:', resp.HomePos)
        print('PosNum:', resp.PosNum)
        print('PosList:')
        for i, p in enumerate(resp.PosList):
            print('  %d: x=%.3f y=%.3f z=%.3f type=%s info=%s' % (i, p.x, p.y, p.z, p.task_type, p.info))
