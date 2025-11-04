#!/usr/bin/env python
import rospy
from tsu200.srv import TaskUpload, TaskUploadResponse
from geometry_msgs.msg import Point


def handle_taskupload(req):
    rospy.loginfo("TaskUpload received: id=%d, HomePos=(%.3f,%.3f,%.3f), PosNum=%d", req.id, req.HomePos.x, req.HomePos.y, req.HomePos.z, req.PosNum)
    for i, p in enumerate(req.PosList):
        rospy.loginfo("  Pos %d: (%.3f,%.3f,%.3f) type=%s info=%s", i, p.x, p.y, p.z, p.task_type, p.info)

    # Basic validation
    if req.PosNum != len(req.PosList):
        msg = 'PosNum does not match length of PosList (%d vs %d)' % (req.PosNum, len(req.PosList))
        rospy.logwarn(msg)
        return TaskUploadResponse(req.id, False, 2, msg)

    # Simulate storing/processing the uploaded task list
    try:
        # (Insert real logic here: save to DB, dispatch tasks, etc.)
        rospy.loginfo('Processing TaskUpload id=%d: %d positions', req.id, req.PosNum)
        # For example, always succeed
        return TaskUploadResponse(req.id, True, 0, 'Uploaded and scheduled')
    except Exception as e:
        rospy.logerr('Error processing TaskUpload: %s', e)
        return TaskUploadResponse(req.id, False, -1, str(e))


def taskupload_server():
    rospy.init_node('taskupload_server')
    s = rospy.Service('task_upload', TaskUpload, handle_taskupload)
    rospy.loginfo('task_upload service ready')
    rospy.spin()


if __name__ == '__main__':
    taskupload_server()
