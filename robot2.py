import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math
import json

class Turtle_2(Node):
    def __init__(self):
        super().__init__("turtle_2")
        self.config_path = "/home/eren/ros_odev/src/my_robot_controller/my_robot_controller/config.json"

        # JSON dosyasını yükle
        self.load_config()
        
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle2/pose", self.pose_control, 10)
        self.speed = 3.0
        self.theta = 0.0  
        self.timer = self.create_timer(0.1, self.move)
        self.at_boundary = False

    def load_config(self):
        try:
            with open(self.config_path, "r") as file:
                config = json.load(file)
                self.turtle2_config = config["turtle2"]

                # Robot2'nin parametrelerini güncelle
                self.speed = self.turtle2_config["linear_speed"]
                self.theta = self.turtle2_config["angular_speed"]

                # Robot2 konfigürasyonunu logla
                self.get_logger().info(f"Turtle2 Config Yüklendi: {self.turtle2_config}")

        except Exception as e:
            self.get_logger().error(f"Config yüklenirken hata oluştu: {e}")

    def save_config(self):
        try:
            with open(self.config_path, "r") as file:
                config = json.load(file)

            # Robot2'nin parametrelerini JSON'a kaydet
            config["turtle2"]["linear_speed"] = self.speed
            config["turtle2"]["angular_speed"] = self.theta

            # JSON dosyasına tekrar kaydet
            with open(self.config_path, "w") as file:
                json.dump(config, file, indent=4)

            self.get_logger().info(f"Turtle2 config başarıyla güncellendi.")

        except Exception as e:
            self.get_logger().error(f"Config kaydedilirken hata oluştu: {e}")


    def pose_control(self, pose: Pose):
        if pose.x >= 11.0 or pose.x <= 0.0 or pose.y >= 11.0 or pose.y <= 0.0:
            self.at_boundary = True
            self.change_direction(pose)
        else:
            self.at_boundary = False

    def move(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = 0.0  

        if self.turtle2_config["linear_speed"] != self.speed or self.turtle2_config["angular_speed"] != self.theta:
            self.save_config()  # Parametreyi kaydet
            self.load_config()

        self.publisher.publish(msg)

    def change_direction(self, pose: Pose):
        strategy = random.choice(["yansima", "random"])

        if strategy == "yansima":
            if pose.x >= 11.0 or pose.x <= 0.0:  
                normal_angle = math.pi / 2  
            else:  
                normal_angle = 0.0  

            self.theta = 2 * normal_angle - pose.theta  

        elif strategy == "random":
            self.theta = random.uniform(30,math.pi)   # rastgele ama 30 derece ile 180 derece arasında sınırlandırdım bu ratgeleliği 

        
        msg = Twist()
        msg.angular.z = self.theta
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Turtle_2()
    rclpy.spin(node)
    rclpy.shutdown()

