#!/usr/bin/env python3
import rospy
from differential_robot.srv import reset

def call_reset_pose(x, y, yaw):
    rospy.wait_for_service('reset_pose')
    try:
        reset_pose = rospy.ServiceProxy('reset_pose', reset)
        resp = reset_pose(x, y, yaw)
        if resp.success:
            rospy.loginfo("Pose reset successful.")
        else:
            rospy.logwarn("Pose reset failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('reset_pose_client')
    call_reset_pose(0.0, 0.0, 0.0)
