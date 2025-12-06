#!/usr/bin/env python
import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from differential_robot.srv import reset, resetResponse

class MazeNavigator:
    def __init__(self):  
        rospy.init_node('maze_navigator')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.path = Path()
        self.path.header.frame_id = "world"
        self.counter = 0
        self.rate = rospy.Rate(10)

        self.base_speed = 0.6   
        self.turn_speed = 0.7
        self.safety_distance = 0.5
        self.wall_follow_distance = 0.3
        self.last_turn_direction = 1
        
        self.reset_service = rospy.Service('reset_pose', reset, self.handle_reset_pose)

        self.run()

    def lidar_callback(self, msg):
        def sector(min_a, max_a):
            idx_min = int((min_a - msg.angle_min) / msg.angle_increment)
            idx_max = int((max_a - msg.angle_min) / msg.angle_increment)
            idx_min = max(0, idx_min)
            idx_max = min(len(msg.ranges)-1, idx_max)
            return [r for r in msg.ranges[idx_min:idx_max+1] if msg.range_min < r < msg.range_max]

        front = sector(-math.pi/6, math.pi/6)
        left = sector(math.pi/12, math.pi/3)
        right = sector(-math.pi/3, -math.pi/12)

        min_front = min(front) if front else float('inf')
        min_left = min(left) if left else float('inf')
        min_right = min(right) if right else float('inf')

        cmd = Twist()
        if min_front < self.safety_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed * (1 if min_left > min_right else -1)
            self.last_turn_direction = 1 if min_left > min_right else -1
        else:
            cmd.linear.x = self.base_speed
            if min_right < 3.0:
                error = min_right - self.wall_follow_distance
                cmd.angular.z = -0.8 * error
            else:
                cmd.angular.z = 0.3 * self.last_turn_direction

        # Simulate odometry
        dt = 0.1
        self.yaw += cmd.angular.z * dt
        self.x += cmd.linear.x * math.cos(self.yaw) * dt
        self.y += cmd.linear.x * math.sin(self.yaw) * dt

        self.cmd_pub.publish(cmd)

    def handle_reset_pose(self, req):
        rospy.loginfo(f"Resetting pose to x={req.x}, y={req.y}, yaw={req.yaw}")
        self.x = req.x
        self.y = req.y
        self.yaw = req.yaw
        self.path = Path()  # Clear the path
        self.path.header.frame_id = "world"
        return resetResponse(success=True)
    
    def run(self):
        while not rospy.is_shutdown():
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0),
                quaternion_from_euler(0, 0, self.yaw),
                rospy.Time.now(),
                "base_link",
                "world"
            )
            if self.counter % 10 == 0:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"
                pose.pose.position = Point(self.x, self.y, 0)
                q = quaternion_from_euler(0, 0, self.yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                self.path.poses.append(pose)
                self.path_pub.publish(self.path)
            self.counter += 1
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MazeNavigator()
    except rospy.ROSInterruptException:
        pass
