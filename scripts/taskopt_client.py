#!/usr/bin/env python
import sys
import rospy
from tsu200.srv import TaskOpt


def call_taskopt(opt, id_val):
    rospy.wait_for_service('task_opt')
    try:
        svc = rospy.ServiceProxy('task_opt', TaskOpt)
        resp = svc(opt, id_val)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s', e)
        return None


if __name__ == '__main__':
    rospy.init_node('taskopt_client')
    opt = sys.argv[1] if len(sys.argv) > 1 else 'noop'
    id_val = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    resp = call_taskopt(opt, id_val)
    if resp:
        print('success:', resp.success)
        print('status_code:', resp.status_code)
        print('message:', resp.message)
