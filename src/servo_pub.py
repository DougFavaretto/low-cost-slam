#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import message_filters
import math

class ServoMapper:
    def __init__(self):
        rospy.init_node('servo_mapper', anonymous=True)
        
        # Publishers
        self.laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # Subscribers com sincronização
        self.ir_sub = message_filters.Subscriber('/lena/ir_sensor', Float32)
        self.servo_sub = message_filters.Subscriber('/lena/ir_motor', UInt16)
        self.imu_sub = message_filters.Subscriber('/lena/heading', Float32)
        
        # Sincronizador de mensagens
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.ir_sub, self.servo_sub, self.imu_sub],
            queue_size=10,
            slop=0.1  # Tolerância de 100ms para sincronização
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Buffers para varredura
        self.distances = []
        self.angles = []
        self.last_servo_pos = -1  # Para detectar fim de varredura
        self.scan_direction = 0  # 0: 0°->180°, 1: 180°->0°
        
        self.rate = rospy.Rate(10)  # 10 Hz para o loop principal
        
    def sync_callback(self, ir_msg, servo_msg, imu_msg):
        # Callback sincronizado para distância, ângulo do servo e yaw
        distance_cm = ir_msg.data
        servo_angle = servo_msg.data
        yaw = imu_msg.data
        
        # Valida distância (limites do Sharp IR: 4-30cm)
        if distance_cm < 4.0 or distance_cm > 30.0:
            rospy.logwarn(f"Invalid distance: {distance_cm} cm")
            return
        
        # Detecta início/fim de varredura
        if self.last_servo_pos != -1:
            if (self.scan_direction == 0 and servo_angle >= 180) or \
               (self.scan_direction == 1 and servo_angle <= 0):
                # Varredura completa, publica LaserScan
                self.publish_scan()
                self.distances = []
                self.angles = []
                self.scan_direction = 1 if self.scan_direction == 0 else 0
                
        # Armazena leitura
        self.distances.append(distance_cm)
        self.angles.append(servo_angle * math.pi / 180.0)  # Converte para radianos
        self.last_servo_pos = servo_angle
        
        if len(self.distances) > 0:
            rospy.loginfo(f"Stored: angle={servo_angle}°, distance={distance_cm}cm")
        
    def publish_scan(self):
        if len(self.distances) < 2:
            rospy.logwarn("Not enough data to publish LaserScan")
            return
        
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser'
        scan.angle_min = 0.0
        scan.angle_max = math.pi  # 180°
        scan.angle_increment = math.pi / (len(self.distances) - 1) if len(self.distances) > 1 else 0.0174533  # 1° em rad
        scan.time_increment = 0.015  # 15ms por passo (ajuste conforme servo_delay no Arduino)
        scan.scan_time = len(self.distances) * 0.015  # Tempo total da varredura
        scan.range_min = 0.04  # 4cm em metros
        scan.range_max = 0.30  # 30cm em metros
        scan.ranges = [d / 100.0 for d in self.distances]  # Converte cm para metros
        scan.intensities = []
        
        self.laser_pub.publish(scan)
        rospy.loginfo(f"Published LaserScan with {len(scan.ranges)} points")
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()  # Mantém o loop ativo, sincronização é feita no callback

if __name__ == '__main__':
    try:
        mapper = ServoMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass