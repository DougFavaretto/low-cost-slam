#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan

class PointToScan:
    def __init__(self):
        rospy.init_node('point_to_scan')
        self.ranges = [float('inf')] * 181  # 0-180 graus
        self.sub_point = rospy.Subscriber('/lena/ir_point', Point, self.point_cb)
        self.sub_servo = rospy.Subscriber('/lena/ir_motor', UInt16, self.servo_cb)
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.last_angle = 0

    def point_cb(self, msg):
        # Servo 0° -> índice 0 (direita)
        # Servo 180° -> índice 180 (esquerda)
        idx = int(self.last_angle)
        dist = (msg.x**2 + msg.y**2)**0.5
        if 0 <= idx < len(self.ranges):
            self.ranges[idx] = dist

    def servo_cb(self, msg):
        self.last_angle = msg.data
        if self.last_angle == 180:  # Fim da varredura
            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "base_link"
            scan.angle_min = -1.5708  # -90° (direita)
            scan.angle_max = 1.5708   # +90° (esquerda)
            scan.angle_increment = 3.14159 / 180
            scan.range_min = 0.05
            scan.range_max = 2.0
            scan.ranges = self.ranges
            self.pub_scan.publish(scan)
            self.ranges = [float('inf')] * 181

if __name__ == '__main__':
    PointToScan()
    rospy.spin()