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
        self.yaw_offset = -1.57  # Valor para corrigir a orientação inicial (~90° em radianos)
        rospy.Subscriber('/lena/heading', Float32, self.cb, queue_size=1)
        self.pub = rospy.Publisher('/lena/odom', Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        rospy.loginfo("Nó de odometria inicializado. Aguardando dados de heading...")

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
            
            # Atualiza orientação
            self.yaw = heading_msg.data / 45  # Usando o valor direto do heading
            
            # Como não temos aceleração, assumimos velocidade constante
            # (ou você pode adicionar um subscriber para velocidade se tiver)
            self.x += self.vx * dt * math.cos(self.yaw)
            self.y += self.vy * dt * math.sin(self.yaw)
            
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
        
        # Se tiver dados de velocidade:
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = (self.last_heading - self.yaw) / 0.1  # aproximação
        
        self.pub.publish(odom)
        self.br.sendTransform((self.x, self.y, 0), q, stamp, 'base_link', 'map')

if __name__ == '__main__':
    rospy.init_node('imu_odom')
    try:
        ImuOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Nó de odometria terminado!")