import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

CAMERA_TOPIC = "/camera/image_raw"
VEL_TOPIC = "/cmd_vel"
QUEUE_SIZE = 10

# State tanımlamaları
class AgvState:
    IDLE = 0
    FOLLOWING = 1
    TURNING = 2
    STOPPED = 3

class AgvNode(Node):
    def __init__(self):
        super().__init__('agv_node')
        
        self.bridge             = CvBridge()
        self.cv_image           = None
        self.contour_center_x   = 0
        self.current_state      = AgvState.IDLE # Başlangıç durumu
        self.turn_count         = 0 # Dönüş sayısını takip etmek için değişken

        # Kamera görüntüsü aboneliği
        self.camera_subscription    = self.create_subscription(
                                            Image, CAMERA_TOPIC, self.image_callback, QUEUE_SIZE
                                        )
        # Hız yayımlayıcı
        self.velocity_publisher     = self.create_publisher(
                                            Twist, VEL_TOPIC, QUEUE_SIZE
                                        )

        self.get_logger().info("AGV node initialized and listening to camera topic.")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_state()

        except Exception as e:
            self.get_logger().error(f"Could not convert from '{msg.encoding}' to 'bgr8': {e}")

    def process_state(self):
        if self.cv_image is None:
            self.send_velocity_command(0.0, 0.0)
            return

        image_to_process = self.cv_image.copy()

        gray_image = cv2.cvtColor(image_to_process, cv2.COLOR_BGR2GRAY)
        _, threshold_image = cv2.threshold(gray_image, 20, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
           self.transition_state(AgvState.STOPPED) # Kontur yoksa STOPPED duruma geç
           self.send_velocity_command(0.0, 0.0)
           self.show_image(image_to_process, 0, 0, True)
           return

        main_contour = max(contours, key=cv2.contourArea)

        width = image_to_process.shape[1]
        middle_x = width // 2
        middle_y = image_to_process.shape[0] // 2 # Orta Y noktası

        self.contour_center_x = self.get_contour_center(main_contour)[0]

        extent = self.get_contour_extent(main_contour)
        
        x, y, w, h = cv2.boundingRect(main_contour)
        corners = [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]
        
        if self.current_state == AgvState.IDLE:
            self.transition_state(AgvState.FOLLOWING)  # Başlangıçta takip moduna geç
        elif self.current_state == AgvState.FOLLOWING:
            linear_speed = 0.3
            angular_speed = (middle_x - self.contour_center_x) * 0.01
            
            if (x,y) == (0,0) and (x, y + h) == (0,480):
                # Orta nokta pikselinin kontur içinde olup olmadığını kontrol et
                if self.is_point_inside_contour(main_contour, (middle_x, middle_y)):
                    self.turn_count += 1  # Dönüş sayısını arttır
                    self.get_logger().info(f"Dönüş Algılandı. Dönüş Sayısı: {self.turn_count}")
                
                self.transition_state(AgvState.TURNING)
                
                linear_speed = 0.0
                angular_speed = 0.0
                
                self.send_velocity_command(linear_speed, angular_speed)  # Dönüşte dur
                return # Dönüşte kal ve başka bir şey yapma

            if extent < 0.2:
                 self.transition_state(AgvState.STOPPED)

            self.send_velocity_command(linear_speed, angular_speed)


        elif self.current_state == AgvState.TURNING:
            # Dönüş mantığı buraya
            # Şimdilik basit bir dönüş
           
           if (x,y) != (0,0) and (x, y + h) != (0,480):
               self.transition_state(AgvState.FOLLOWING) # Dönüş bittiğinde takip etme durumuna geç
               
        elif self.current_state == AgvState.STOPPED:
             self.send_velocity_command(0.0, 0.0)
        
        self.show_image(image_to_process, middle_x, main_contour)


    def show_image(self,image_to_process, middle_x, main_contour, stop = False):
      if not stop:
        cv2.drawContours(image_to_process, [main_contour], -1, (0, 255, 0), 3)
        cv2.circle(image_to_process, (self.contour_center_x, image_to_process.shape[0] // 2), 7, (255, 255, 255), -1)
        cv2.circle(image_to_process, (middle_x, image_to_process.shape[0] // 2), 3, (0, 0, 255), -1)
      cv2.imshow("Processed Image", image_to_process)
      cv2.waitKey(1)

    def transition_state(self, new_state):
        self.current_state = new_state
        state_names = {
        AgvState.IDLE: "IDLE",
        AgvState.FOLLOWING: "FOLLOWING",
        AgvState.TURNING: "TURNING",
        AgvState.STOPPED: "STOPPED"
        }
        self.get_logger().info(f"Transitioned to state: {state_names[new_state]}")


    def send_velocity_command(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.velocity_publisher.publish(twist_msg)

    def get_contour_center(self, contour):
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return 0, 0
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        return cx, cy

    def get_contour_extent(self, contour):
        area = cv2.contourArea(contour)
        bounding_rect = cv2.boundingRect(contour)
        rect_area = bounding_rect[2] * bounding_rect[3]
        return area / rect_area if rect_area > 0 else 0

    def is_point_inside_contour(self, contour, point):
         return cv2.pointPolygonTest(contour, point, False) >= 0

    def stop(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    agv_node = AgvNode()

    try:
        rclpy.spin(agv_node)
    except KeyboardInterrupt:
        pass

    agv_node.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()