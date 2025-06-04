#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import math
import tf

class SimpleMapper:
    def __init__(self):
        rospy.init_node('simple_mapper')
        
        # Parâmetros do mapa
        self.resolution = 0.05  # 5cm/cell
        self.width = 200        # 10m
        self.height = 200       # 10m
        self.origin_x = -5.0    # -5m
        self.origin_y = -5.0    # -5m
        
        # Mapa inicial (50% desconhecido)
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        
        # Posição atual do robô
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.sub_odom = rospy.Subscriber('/lena/odom', Odometry, self.odom_cb)
        self.pub = rospy.Publisher('/simple_map', OccupancyGrid, queue_size=1)
        
        rospy.loginfo("Simple Mapper ready")
    
    def odom_cb(self, msg):
        # Atualiza a posição do robô
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extrai a orientação (theta) dos quaternions
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.robot_theta = tf.transformations.euler_from_quaternion(quaternion)
        
        # Atualiza o mapa com a nova posição
        self.update_robot_position()
        
        # Publica o mapa atualizado
        self.publish_map()
    
    def update_robot_position(self):
        # Converte a posição do robô para coordenadas do grid
        grid_x = int((self.robot_x - self.origin_x) / self.resolution)
        grid_y = int((self.robot_y - self.origin_y) / self.resolution)
        
        # Verifica se está dentro dos limites do mapa
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            # Marca a posição do robô como ocupada (100)
            self.grid[grid_y, grid_x] = 100
            
            # Marca células vizinhas como livres (0)
            radius = 3  # células ao redor
            for i in range(-radius, radius+1):
                for j in range(-radius, radius+1):
                    nx, ny = grid_x + i, grid_y + j
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # Não sobrescrever a posição do robô
                        if not (i == 0 and j == 0):
                            self.grid[ny, nx] = 0
    
    def scan_cb(self, msg):
        # Processa o scan com base na posição atual do robô
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        for i, distance in enumerate(msg.ranges):
            if not np.isfinite(distance):
                continue
                
            if distance > msg.range_max or distance < msg.range_min:
                continue
                
            # Calcula a posição real do ponto detectado
            angle = angles[i] + self.robot_theta
            x = self.robot_x + distance * math.cos(angle)
            y = self.robot_y + distance * math.sin(angle)
            
            # Converte para coordenadas do grid
            grid_x = int((x - self.origin_x) / self.resolution)
            grid_y = int((y - self.origin_y) / self.resolution)
            
            # Verifica se está dentro dos limites do mapa
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                # Marca o ponto como ocupado
                self.grid[grid_y, grid_x] = 100
                
                # Raycasting - marca células entre o robô e o obstáculo como livres
                self.mark_free_cells(self.robot_x, self.robot_y, x, y)
        
        self.publish_map()
    
    def mark_free_cells(self, x0, y0, x1, y1):
        # Implementação de Bresenham para traçar linha
        grid_x0 = int((x0 - self.origin_x) / self.resolution)
        grid_y0 = int((y0 - self.origin_y) / self.resolution)
        grid_x1 = int((x1 - self.origin_x) / self.resolution)
        grid_y1 = int((y1 - self.origin_y) / self.resolution)
        
        dx = abs(grid_x1 - grid_x0)
        dy = -abs(grid_y1 - grid_y0)
        sx = 1 if grid_x0 < grid_x1 else -1
        sy = 1 if grid_y0 < grid_y1 else -1
        err = dx + dy
        
        while True:
            # Não marca o ponto final (obstáculo)
            if grid_x0 == grid_x1 and grid_y0 == grid_y1:
                break
                
            if 0 <= grid_x0 < self.width and 0 <= grid_y0 < self.height:
                self.grid[grid_y0, grid_x0] = 0  # Célula livre
                
            e2 = 2 * err
            if e2 >= dy:
                if grid_x0 == grid_x1:
                    break
                err += dy
                grid_x0 += sx
            if e2 <= dx:
                if grid_y0 == grid_y1:
                    break
                err += dx
                grid_y0 += sy
    
    def publish_map(self):
        # Cria e publica a mensagem OccupancyGrid
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        
        # Flatten o grid para publicação
        map_msg.data = self.grid.flatten().tolist()
        
        self.pub.publish(map_msg)

if __name__ == '__main__':
    try:
        mapper = SimpleMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass