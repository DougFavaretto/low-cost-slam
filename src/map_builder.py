#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MapBuilder:
    def __init__(self):
        rospy.init_node('map_builder')
        self.path_points = []
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=10)
        rospy.Subscriber('/lena/odom', Odometry, self.odom_cb)
        rospy.loginfo("Map Builder inicializado. Coletando pontos...")

    def odom_cb(self, msg):
        # Armazena posição atual
        point = Point()
        point.x = msg.pose.pose.position.x
        point.y = msg.pose.pose.position.y
        self.path_points.append(point)
        
        # Publica marcador RViz
        self.publish_path()

    def publish_path(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Largura da linha
        marker.color.a = 1.0   # Opacidade
        marker.color.r = 1.0   # Cor vermelha
        marker.points = self.path_points
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        MapBuilder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass