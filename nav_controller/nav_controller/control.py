import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys , threading , time
from rclpy.qos import QoSProfile
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

#------------------------VARIABLES---------------------------------#
expansionSize = 10

# -----------------------------------------------------------------#
def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid
# Harita üzerinde bulunan engelleri genişletmek için kullanılır
def expandObstacles(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    obstacles = np.where(data == 100)
    for i in range(-expansionSize,expansionSize+1):
        for j in range(-expansionSize,expansionSize+1):
            if i  == 0 and j == 0:
                continue
            x = obstacles[0]+i
            y = obstacles[1]+j
            x = np.clip(x,0,height-1) # X ekseninde haritadan taşma olmasını engelliyor
            y = np.clip(y,0,width-1)  # Y ekseninde haritadan taşma olmasını engelliyor
            data[x,y] = 100
    data = data*resolution
    return data


#Keşfedilmemiş noktaları bulmak için kullanılır
def findBoundaryPoints(data):
    for i, row in enumerate(data):
        for j, value in enumerate(row):
            if value == 0.0:
                # Komşu hücre değerlerini kontrol et
                if (i > 0 and data[i - 1][j] < 0) or \
                   (i < len(data) - 1 and data[i + 1][j] < 0) or \
                   (j > 0 and data[i][j - 1] < 0) or \
                   (j < len(data[i])-1 and data[i][j + 1] < 0):
                    data[i][j] = 2
    return data



#0 bilinen, -1 bilinmeyen, 100 engel
def f_point(data,width,height,resolution,vehicle_x,vehicle_y,originX,originY):
    data = expandObstacles(data,width,height,resolution)
    
    data[data>1] = 1
    data[data<0] = -1
    
    data = findBoundaryPoints(data)
    #data[vehicle_y][vehicle_x] = 100
    data,groups = assign_groups(data) #Sınır noktaları gruplandır
    groups = fGroups(groups) #Grupları küçükten büyüğe sırala. En buyuk 5 grubu al
    data, goal_x, goal_y = findClosestGroup(data,groups,(vehicle_y,vehicle_y),resolution,originX,originY)
    return data, goal_x, goal_y
                    
    
def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups

def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups) # sağ alt çapraz
    dfs(matrix, i - 1, j - 1, group, groups) # sol üst çapraz
    dfs(matrix, i - 1, j + 1, group, groups) # sağ üst çapraz
    dfs(matrix, i + 1, j - 1, group, groups) # sol alt çapraz
    return group + 1

def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups


def findClosestGroup(matrix,groups, current,resolution,originX,originY):

    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        goal_y = middle[0]
        goal_x = middle[1]
        matrix[middle[0]][middle[1]] = 1
    return matrix, goal_x, goal_y
#-------------------------------------------------------------------#


class AgvAutomation(Node):
    def __init__(self):
        super().__init__('agv_automation')

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.occupancy_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_callback,
            10)
        
        self.odometry_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            QoSProfile(depth=10))

        self.publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10)
        
        self.safety_margin_side = 0.5 
        self.safety_margin_front_rear = 0.66 
        self.reset_proxy = self.create_client(Empty, "/reset_world")


        self.publisher1 = self.create_publisher(MarkerArray, "goal_point", 3)
        self.publisher2 = self.create_publisher(MarkerArray, "linear_velocity", 1)
        self.publisher3 = self.create_publisher(MarkerArray, "angular_velocity", 1)

        # Hedef noktaları gruplandırırken recursion limitini arttırmak için gerekli
        sys.setrecursionlimit(3000)

        # Haritanın görselleştirilmesi için thread kullanımı
        threading.Thread(target=self.my_thread).start()


    def goal_pose_callback(self,msg):

        self.goal = (msg.pose.position.x,msg.pose.position.y)       
        self.goal_yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        
 

    def odom_callback(self,msg):

        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)



    def lidar_callback(self, msg):

        self.scan_data = msg
        self.scan = msg.ranges
        self.collision_detect()
        

    def occupancy_callback(self, msg):

        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

        
    def publish_markers(self):
            # Publish visual data in Rviz
            markerArray = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.real_world_x
            marker.pose.position.y = self.real_world_y
            marker.pose.position.z = 0.0
            markerArray.markers.append(marker)

            self.publisher1.publish(markerArray)

    def my_thread(self):
        plt.ion()  # Matplotlib'in interaktif modunu açın
        fig, ax = plt.subplots()

        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'odom_data') or not hasattr(self, 'scan_data'):
                time.sleep(0.1)
                continue

            vehicle_x = int((self.x - self.originX) / self.resolution)
            vehicle_y = int((self.y - self.originY) / self.resolution)
            
            start_time = time.time()

            # Haritayı işleyin
            my_map, self.goal_x, self.goal_y = f_point(self.data, self.width, self.height, self.resolution, vehicle_x, vehicle_y,self.originX,self.originY)
            self.real_world_x = self.goal_x * self.resolution + self.originX
            self.real_world_y = self.goal_y * self.resolution + self.originY
            self.publish_markers()
            # Önceki çizimi temizleyin
            ax.clear()

            # Haritayı çizdirin
            ax.imshow(my_map, cmap="gray")
            print(self.real_world_x)
            print(self.real_world_y)
            # Çizimi güncelleyin
            plt.draw()
            plt.pause(0.01)  # Çizim için kısa bir duraklama

            # Bellekte tutulan nesneleri temizleyin
            del my_map





    def collision_detect(self):

        if not hasattr(self, 'scan'):
            self.get_logger().warn("No ranges available from LiDAR.")
            return
        
        # Çarpışma tespiti için minimum uzaklığın alınması
        min_rear = min(self.scan[0:61] + self.scan[299:360], default=float('inf'))
        min_front = min(self.scan[119:241], default=float('inf'))
        min_left = min(self.scan[241:299], default=float('inf'))
        min_right = min(self.scan[61:119], default=float('inf'))

        self.collision = False

        # Çarpışma tespiti
        if min_front < self.safety_margin_front_rear:
            self.get_logger().info(f"Önde çarpışma riski! Mesafe: {min_front:.2f} m")
            self.collision = True

        if min_left < self.safety_margin_side:
            self.get_logger().info(f"Solda çarpışma riski! Mesafe: {min_left:.2f} m")
            self.collision = True

        if min_right < self.safety_margin_side:
            self.get_logger().info(f"Sağda çarpışma riski! Mesafe: {min_right:.2f} m")
            self.collision = True

        if min_rear < self.safety_margin_front_rear:
            self.get_logger().info(f"Arkada çarpışma riski! Mesafe: {min_rear:.2f} m")
            self.collision = True
        
        # Simülasyonun sıfırlanması
        if self.collision:
            self.reset_simulation()



    def reset_simulation(self):
            
            while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('reset : service not available, waiting again...')

            try:
                self.reset_proxy.call_async(Empty.Request())
            except rclpy.ServiceException as e:
                self.get_logger().error("/gazebo/reset_simulation service call failed")

    
    

def main(args=None):
    rclpy.init(args=args)
    agv_automation = AgvAutomation()
    rclpy.spin(agv_automation)
    agv_automation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
