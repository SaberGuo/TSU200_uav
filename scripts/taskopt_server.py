#!/usr/bin/env python
import rospy
from tsu200.srv import TaskOpt, TaskOptResponse


def handle_taskopt(req):
    rospy.loginfo("TaskOpt received: opt='%s', id=%d", req.opt, req.id)
    # Simulate performing the requested opt; replace with real logic
    try:
        # Example: if opt == "noop" just succeed, otherwise pretend success too
        success = True
        status_code = 0
        message = "Executed: {}".format(req.opt)
    except Exception as e:
        success = False
        status_code = -1
        message = str(e)

    rospy.loginfo("TaskOpt result: success=%s, code=%d, msg=%s", success, status_code, message)
    return TaskOptResponse(success, status_code, message)


def taskopt_server():
    rospy.init_node('taskopt_server')
    s = rospy.Service('task_opt', TaskOpt, handle_taskopt)
    rospy.loginfo('task_opt service ready')
    rospy.spin()


if __name__ == '__main__':
    taskopt_server()
