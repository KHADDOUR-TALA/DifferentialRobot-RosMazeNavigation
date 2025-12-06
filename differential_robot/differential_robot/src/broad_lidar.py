#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math
import tf
import numpy as np
from tf.broadcaster import TransformBroadcaster

class SimulatedLidar:
    def __init__(self):  
        rospy.init_node('simulated_lidar')
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = TransformBroadcaster()  
        self.rate = rospy.Rate(10)
        
        self.lidar_offset = (0.1, 0.0, 0.175)  # x, y, z

        self.walls = [

    (-5, -5, 5, -5),
    (5, -5, 5, 5),
    (5, 5, -1, 5),  
    (-5, 5, -5, -5),

    (-4, -4, 4, -4),
    (4, -4, 4, 3),  
    (4, 4, -4, 4),
    (-4, 4, -4, -4),

    (-3, -3, 3, -3),
    (3, -3, 3, 3),
    (-3, -3, -3, 3),
    (-3, 3, 2, 3),  

    (-2, -2, 2, -2),
    (2, -2, 2, 2),
    (-2, -2, -2, 1),  
    (-2, 2, 2, 2),


    (-1, -1, 1, -1),
    (1, -1, 1, 1),
    (1, 1, -1, 1),
        ]

    def publish_lidar_transform(self):
        """Publish transform from base_link to hokuyo_link"""
        self.tf_broadcaster.sendTransform(
            (self.lidar_offset[0], self.lidar_offset[1], self.lidar_offset[2]),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "hokuyo_link",
            "base_link"
        )

    def get_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('world', 'base_link', rospy.Time(0))
            return trans[0], trans[1], tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None

    def ray_cast(self, x0, y0, angle):
        min_dist = 10.0
        x1 = x0 + min_dist * math.cos(angle)
        y1 = y0 + min_dist * math.sin(angle)

        for (x2, y2, x3, y3) in self.walls:
            denom = (x1 - x0) * (y3 - y2) - (y1 - y0) * (x3 - x2)
            if abs(denom) < 1e-8:
                continue
            t = ((x2 - x0) * (y3 - y2) - (y2 - y0) * (x3 - x2)) / denom
            u = -((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)) / denom
            if 0 <= t <= 1 and 0 <= u <= 1:
                intersect_x = x0 + t * (x1 - x0)
                intersect_y = y0 + t * (y1 - y0)
                dist = math.hypot(intersect_x - x0, intersect_y - y0)
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def run(self):
        while not rospy.is_shutdown():

            self.publish_lidar_transform()
            
            pose = self.get_robot_pose()
            if pose is None:
                self.rate.sleep()
                continue

            x, y, yaw = pose

            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "hokuyo_link"  
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = math.radians(1)
            scan.range_min = 0.1
            scan.range_max = 10.0
            scan.ranges = []

            angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
            for a in angles:
                world_angle = yaw + a
                r = self.ray_cast(x, y, world_angle)
                scan.ranges.append(r)

            self.pub.publish(scan)
            self.rate.sleep()

if __name__ == '__main__':
    SimulatedLidar().run()
