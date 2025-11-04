#!/usr/bin/env python
import sys
import rospy
from tsu200.srv import Compute


def compute_client(a, b):
    rospy.wait_for_service('compute')
    try:
        compute = rospy.ServiceProxy('compute', Compute)
        resp = compute(a, b)
        return resp.sum
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":
    rospy.init_node('compute_client')
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    result = compute_client(a, b)
    print("Result:", result)
