#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

class ServoMapper:
    def __init__(self):
        rospy.init_node('servo_pub', anonymous=True)
        
        self.ir_motor = rospy.Publisher('/lena/ir_motor', UInt16, queue_size=5)
        self.laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        self.ir_sub = rospy.Subscriber('/lena/ir_sensor', Float32, self.ir_callback)
        
        self.distances = []
        self.angles = []
        self.rate = rospy.Rate(3)  # 5 Hz
        
    def ir_callback(self, data):
        # Store the latest distance measurement
        self.current_distance = data.data
        rospy.loginfo(f"Received distance: {self.current_distance}")
        
    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser'
        scan.angle_min = 0.0
        scan.angle_max = 3.14159  # 180°
        scan.angle_increment = 3.14159 / (len(self.distances)-1) if len(self.distances) > 1 else 1.0
        scan.time_increment = 0.2
        scan.scan_time = 1.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [d/100.0 for d in self.distances]  # Convertendo cm para metros
        scan.intensities = []
        
        self.laser_pub.publish(scan)
        rospy.loginfo(f"Published scan with ranges: {scan.ranges}")
    
    def run(self):
        positions = [0, 45, 90, 135, 180]
        
        while not rospy.is_shutdown():
            self.distances = []
            self.angles = []
            
            for pos in positions:
                rospy.loginfo(f"Moving servo to position: {pos}")
                self.ir_motor.publish(pos)
                self.rate.sleep()
                
                if hasattr(self, 'current_distance'):
                    self.distances.append(self.current_distance)
                    self.angles.append(pos)
                else:
                    rospy.logwarn("No distance measurement received yet")
                
            if len(self.distances) > 0:
                self.publish_scan()
            else:
                rospy.logwarn("No valid distance measurements to publish")

if __name__ == '__main__':
    try:
        mapper = ServoMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
