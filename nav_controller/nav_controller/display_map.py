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
        

        # Haritanın görselleştirilmesi için thread kullanımı
        threading.Thread(target=self.display_occupancy_grid).start()


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

        

    def occupancy_callback(self, msg):

        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data
    
        
   


    def display_occupancy_grid(self):

        while True:

            if not hasattr(self, 'map_data') or not hasattr(self, 'odom_data') or not hasattr(self, 'scan_data'):
                time.sleep(0.1)
                continue

            # Haritanın oluşturulması
            grid_data = np.array(self.data).reshape((self.height, self.width))
            self.normalized_grid = 255 - (grid_data / 100.0 * 255).astype(np.uint8)
            
            # Siyah beyaz formattaki haritayı renkli hale getirme
            self.bgr_grid = cv2.cvtColor(self.normalized_grid, cv2.COLOR_GRAY2BGR)


            
            # Aracın konumunun haritaya göre ölçeklenmesi
            vehicle_x = int((self.x - self.originX) / self.resolution)
            vehicle_y = int((self.y - self.originY) / self.resolution)

        
            # Araç ve hedef konum için kullanılacak çemberlerin yarıçapları
            vehicle_radius = 10
            goal_radius = 3

            # Aracın konumun anlık olarak haritada gösterilmesi
            if 0 <= vehicle_x < self.width and 0 <= vehicle_y < self.height:
                cv2.circle(self.bgr_grid, (vehicle_x, vehicle_y), vehicle_radius, (0, 255, 255), -1) 

            # Rviz üzerinden hedef konum gönderilmesini bekle    
            if hasattr(self, 'goal'):
                
                # Gönderilne hedef konumların haritaya göre ölçeklenmesi
                goal_x = int((self.goal[0] - self.originX) / self.resolution)
                goal_y = int((self.goal[1] - self.originY) / self.resolution)

                # Rviz üzerinden gönderilen hedef konum ile araç arasındaki çizginin yönünü hesapla
                direction_x = goal_x - vehicle_x
                direction_y = goal_y - vehicle_y
                direction_length = math.sqrt(direction_x**2 + direction_y**2)


                try:
                    # Çizginin başlangıç noktasını çemberin dışına taşımak için normalize et
                    start_x = int(vehicle_x + (direction_x / direction_length) * 10)
                    start_y = int(vehicle_y + (direction_y / direction_length) * 10)
                except rclpy.ServiceException as e:
                    self.get_logger().error("Math Error")


                # İki merkez arasındaki mesafeyi hesapla
                distance = math.sqrt((goal_x - vehicle_x) ** 2 + (goal_y - vehicle_y) ** 2)

                # Kesişimi kontrol et
                if distance >= vehicle_radius + goal_radius:

                    cv2.circle(self.bgr_grid, (goal_x, goal_y), goal_radius, (0, 255, 0), -1) 

                    cv2.line(self.bgr_grid, (start_x, start_y), (goal_x, goal_y), (0, 255, 0), thickness=1)
                else:
                    self.flag = 0
            
            
            # Haritanın ters çervrilmesi
            self.bgr_grid = cv2.flip(self.bgr_grid,0)

            #cv2.putText(bgr_grid,str(math.degrees(self.yaw)),(15,15),cv2.FONT_HERSHEY_SIMPLEX,1,(255, 0, 0),2,cv2.LINE_AA)


            
            # Haritayı göster
            cv2.imshow("Occupancy Grid", self.bgr_grid)
            
            if cv2.waitKey(1) & 0xFF == 27:  # Çıkmak için 'ESC' bas
                break

        cv2.destroyAllWindows()


        




    
    

def main(args=None):
    rclpy.init(args=args)
    agv_automation = AgvAutomation()
    rclpy.spin(agv_automation)
    agv_automation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
