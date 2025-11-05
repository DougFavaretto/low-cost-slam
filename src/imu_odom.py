#!/usr/bin/env python3
import rospy, math, tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

class ImuOdom:
    def __init__(self):
        self.x = self.y = self.vx = self.vy = self.yaw = 0.0
        self.t_prev = None
        self.last_heading = 0.0
        self.yaw_offset = 0.0  # Offset será ajustado no processamento do heading
        
        # Variáveis para o encoder
        self.last_ticks = None
        self.tick_time_prev = None
        self.wheel_radius = 0.035  # Raio da roda em metros (ajuste conforme necessário)
        self.gear_reduction = 48   # Redução do encoder
        self.ticks_per_revolution = 6  # Ticks por revolução do encoder (ajuste conforme necessário)
        
        rospy.Subscriber('/lena/heading', Float32, self.cb, queue_size=1)
        rospy.Subscriber('/lena/raw/right_ticks', Float32, self.encoder_cb, queue_size=1)
        self.pub = rospy.Publisher('/lena/odom', Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        rospy.loginfo("Nó de odometria inicializado. Aguardando dados de heading e encoder...")

    def encoder_cb(self, tick_msg):
        """Callback para processar dados do encoder e calcular velocidade linear"""
        now = rospy.Time.now()
        
        if self.last_ticks is None or self.tick_time_prev is None:
            self.last_ticks = tick_msg.data
            self.tick_time_prev = now
            return
            
        try:
            dt = (now - self.tick_time_prev).to_sec()
            if dt <= 0:
                return
                
            # Calcula diferença de ticks
            tick_diff = tick_msg.data - self.last_ticks
            self.last_ticks = tick_msg.data
            self.tick_time_prev = now
            
            # Converte ticks para distância percorrida
            # tick_diff -> revoluções do encoder -> revoluções da roda -> distância
            encoder_revolutions = tick_diff / self.ticks_per_revolution
            wheel_revolutions = encoder_revolutions / self.gear_reduction
            distance = wheel_revolutions * 2 * math.pi * self.wheel_radius
            
            # Calcula velocidade linear
            self.vx = distance / dt
            
        except Exception as e:
            rospy.logerr(f"Erro ao processar encoder: {str(e)}")

    def cb(self, heading_msg):
        now = rospy.Time.now()
        
        if not self.t_prev:
            self.t_prev = now
            self.last_heading = heading_msg.data
            return
            
        try:
            dt = (now - self.t_prev).to_sec()
            self.t_prev = now
            
            # Calcula a velocidade angular (diferença entre headings)
            angular_z = (heading_msg.data - self.last_heading) / dt
            self.last_heading = heading_msg.data
            
            # Atualiza orientação - converte de graus para radianos e ajusta offset
            # Se 90° é frente, subtraimos 90° e convertemos para radianos
            heading_degrees = heading_msg.data - 90.0  # Ajusta para que 0° seja a frente
            self.yaw = math.radians(heading_degrees)  # Converte para radianos
            
            # Atualiza posição usando velocidade linear do encoder
            self.x += self.vx * dt * math.cos(self.yaw)
            self.y += self.vx * dt * math.sin(self.yaw)  # Mudança: usando vx em vez de vy
            
            self.publish(now)
            
        except Exception as e:
            rospy.logerr(f"Erro ao processar heading: {str(e)}")

    def publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)
        
        # Velocidades baseadas no encoder e IMU
        odom.twist.twist.linear.x = self.vx  # Velocidade linear do encoder
        odom.twist.twist.linear.y = 0.0     # Robô diferencial, sem movimento lateral
        odom.twist.twist.angular.z = (self.last_heading - self.yaw) / 0.1  # Velocidade angular aproximada
        
        self.pub.publish(odom)
        self.br.sendTransform((self.x, self.y, 0), q, stamp, 'base_link', 'map')

if __name__ == '__main__':
    rospy.init_node('imu_odom')
    try:
        ImuOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Nó de odometria terminado!")