#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_maze():
    rospy.init_node('maze_marker_publisher')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)

    walls = [

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

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "maze"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0

        for (x1, y1, x2, y2) in walls:
            marker.points.append(Point(x=x1, y=y1, z=0))
            marker.points.append(Point(x=x2, y=y2, z=0))

        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    publish_maze()
