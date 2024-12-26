import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import json


class Turtle_1(Node):
    def __init__(self):
        super().__init__("turtle_follower")
        self.config_path = "/home/eren/ros_odev/src/my_robot_controller/my_robot_controller/config.json"

        # JSON dosyasını yükle
        self.load_config()

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber_1 = self.create_subscription(Pose, '/turtle1/pose', self.pose_anlik, 10)
        self.pose_subscriber_2 = self.create_subscription(Pose, '/turtle2/pose', self.pose_hedef, 10)

        self.anlik_pose = None
        self.hedef_pose = None

        self.timer = self.create_timer(0.1, self.hedef_takibi)

        # PID parametreleri
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.5
        self.Kp_angular = 6.0
        self.integral_linear = 0.0
        self.önceki_error_linear = 0.0
        self.hedef_uzaklik = 2.0 

    def load_config(self):
        try:
            with open(self.config_path, "r") as file:
                config = json.load(file)
                self.turtle1_config = config["turtle1"]

                # Robot1'in parametrelerini güncelle
                self.Kp = self.turtle1_config["Kp"]
                self.Ki = self.turtle1_config["Ki"]
                self.Kd = self.turtle1_config["Kd"]
                self.Kp_angular = self.turtle1_config["Kp_angular"]
                self.hedef_uzaklik = self.turtle1_config["hedef_uzaklik"]
                self.max_linear_speed = self.turtle1_config["max_linear_speed"]

                # Robot1 konfigürasyonunu logla
                self.get_logger().info(f"Turtle1 Config Yüklendi: {self.turtle1_config}")


        except Exception as e:
            self.get_logger().error(f"Config yüklenirken hata oluştu: {e}")

    def save_config(self):
        try:
            with open(self.config_path, "r") as file:
                config = json.load(file)

            # Robot1'in parametrelerini JSON'a kaydet
            config["turtle1"]["Kp"] = self.Kp
            config["turtle1"]["Ki"] = self.Ki
            config["turtle1"]["Kd"] = self.Kd
            config["turtle1"]["Kp_angular"] = self.Kp_angular
            config["turtle1"]["hedef_uzaklik"] = self.hedef_uzaklik
            config["turtle1"]["max_linear_speed"] = self.max_linear_speed

            # JSON dosyasına tekrar kaydet
            with open(self.config_path, "w") as file:
                json.dump(config, file, indent=4)

            self.get_logger().info(f"Turtle1 config başarıyla güncellendi.")

        except Exception as e:
            self.get_logger().error(f"Config kaydedilirken hata oluştu: {e}")


    def pose_anlik(self, pose: Pose):
        self.anlik_pose = pose
        #self.get_logger().info(f"Turtle1 Pose: x={pose.x}, y={pose.y}, theta={pose.theta}")

    def pose_hedef(self, pose: Pose):
        self.hedef_pose = pose
        #self.get_logger().info(f"Turtle2 Pose: x={pose.x}, y={pose.y}, theta={pose.theta}")

    def hedef_takibi(self):
        if self.anlik_pose is None or self.hedef_pose is None:
            self.get_logger().info("Pozisyon bilgisi bekleniyor...")
            return
        
        if self.turtle1_config["Kp"] != self.Kp or  self.turtle1_config["Ki"] != self.Ki or self.turtle1_config["Kd"] != self.Kd or self.turtle1_config["Kp_angular"] != self.Kp_angular or self.turtle1_config["hedef_uzaklik"] != self.hedef_uzaklik or self.turtle1_config["max_linear_speed"] != self.max_linear_speed:
            self.save_config()  # Parametreyi kaydet
            self.load_config()

        dx = self.hedef_pose.x - self.anlik_pose.x
        dy = self.hedef_pose.y - self.anlik_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        error_linear = distance - self.hedef_uzaklik

        self.integral_linear += error_linear
        derivative_linear = error_linear - self.önceki_error_linear
        control_signal_linear = (
            self.Kp * error_linear +
            self.Ki * self.integral_linear +
            self.Kd * derivative_linear
        )
        self.önceki_error_linear = error_linear

        target_theta = math.atan2(dy, dx)
        angular_error = target_theta - self.anlik_pose.theta

        # Açıyı [-pi, pi] aralığına normalize etme işlemleri
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        control_signal_angular = self.Kp_angular * angular_error

        msg = Twist()
        msg.linear.x = max(0.0, min(control_signal_linear, 2.0))  # Maksimum hız limiti koydum
        msg.angular.z = control_signal_angular
        self.publisher.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = Turtle_1()
    rclpy.spin(node)
    rclpy.shutdown()

