#!/usr/bin/env python
#!coding:utf-8
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def main():
    rospy.init_node("points_and_lines", anonymous=True)
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(30)
    f = 0.0
    while not rospy.is_shutdown():
        types = [Marker.POINTS, Marker.LINE_STRIP, Marker.LINE_LIST]
        markers = [Marker() for _ in ['points', 'line_strip', 'line_list']]
        for i, m in enumerate(markers):  # 分别处理不同的marker
            m.header.frame_id = '/my_frame'
            m.header.stamp = rospy.Time.now()
            m.ns = 'points_and_lines'
            m.pose.orientation.w = 1.0
            m.action = Marker.ADD
            m.id = i
            m.type = types[i]
            m.color.a = 1.0
            if i == 0:  # point
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.color.g = 1.0
            elif i == 1:  # line_strip
                m.scale.x = 0.1
                m.color.b = 1.0
            elif i == 2:
                m.scale.x = 0.1
                m.color.r = 1.0
        for i in range(100):  # 给每一个marker添加points
            y = 5 * np.sin(f + i/100.0 * 2 * np.pi)
            z = 5 * np.cos(f + i/100.0 * 2 * np.pi)
            p = Point(i-50, y, z)
            markers[0].points.append(p)
            markers[1].points.append(p)
            markers[2].points.extend([p, Point(i-50, y, z+1)])
        for m in markers:
            pub.publish(m)
        rate.sleep()
        f += 0.04


if __name__ == '__main__':
    main()
