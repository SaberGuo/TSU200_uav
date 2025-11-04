#!/usr/bin/env python
import rospy
from tsu200.srv import Compute, ComputeResponse


def handle_compute(req):
    rospy.loginfo("Compute request: %d + %d", req.a, req.b)
    return ComputeResponse(req.a + req.b)


def compute_server():
    rospy.init_node('compute_server')
    s = rospy.Service('compute', Compute, handle_compute)
    rospy.loginfo("Ready to compute")
    rospy.spin()


if __name__ == '__main__':
    compute_server()
