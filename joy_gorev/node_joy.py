#!/usr/bin/env python3
import rclpy as rcl
from rclpy.node import Node
from sensor_msgs.msg import Joy
from example_interfaces.msg import Int32
import math

class joy(Node):
    def __init__(self):
        super().__init__("Publisher")

        self.width  = 84.0
        self.length = 113.0

        self.create_subscription(Joy, "joy", self.msg, 10)

        self.angle_fl = self.create_publisher(Int32, "wheel_front_left_angle", 10)
        self.angle_fr = self.create_publisher(Int32, "wheel_front_right_angle", 10)
        self.angle_rl = self.create_publisher(Int32, "wheel_rear_left_angle", 10)
        self.angle_rr = self.create_publisher(Int32, "wheel_rear_right_angle", 10)

        self.speed_fl = self.create_publisher(Int32, "wheel_front_left_speed", 10)
        self.speed_fr = self.create_publisher(Int32, "wheel_front_right_speed", 10)
        self.speed_rl = self.create_publisher(Int32, "wheel_rear_left_speed", 10)
        self.speed_rr = self.create_publisher(Int32, "wheel_rear_right_speed", 10)

        self.get_logger().info("Basladi")

    def msg(self, data1: Joy):
        base_speed = int(data1.axes[1] * 63)

        steer = data1.axes[0]
        max_steer = math.radians(35)
        delta = steer * max_steer
        eps = 1e-3

        if abs(delta) < eps:
            FL = FR = RL = RR = 180
            v_fl = v_fr = v_rl = v_rr = base_speed
        else:
            R = self.length / math.tan(abs(delta))

            if delta > 0:
                FL_rad = math.atan(self.length / (R - self.width / 2))
                FR_rad = math.atan(self.length / (R + self.width / 2))
                R_fl = R - self.width / 2
                R_fr = R + self.width / 2
            else:
                FL_rad = -math.atan(self.length / (R + self.width / 2))
                FR_rad = -math.atan(self.length / (R - self.width / 2))
                R_fl = R + self.width / 2
                R_fr = R - self.width / 2

            FL = math.degrees(FL_rad) + 180
            FR = math.degrees(FR_rad) + 180
            RL = 180
            RR = 180

            v_fl = base_speed * (R_fl / R)
            v_fr = base_speed * (R_fr / R)
            v_rl = v_fl
            v_rr = v_fr

        msg = Int32()

        msg.data = int(FL)
        self.angle_fl.publish(msg)

        msg.data = int(FR)
        self.angle_fr.publish(msg)

        msg.data = int(RL)
        self.angle_rl.publish(msg)

        msg.data = int(RR)
        self.angle_rr.publish(msg)

        msg.data = int(v_fl)
        self.speed_fl.publish(msg)

        msg.data = int(v_fr)
        self.speed_fr.publish(msg)

        msg.data = int(v_rl)
        self.speed_rl.publish(msg)

        msg.data = int(v_rr)
        self.speed_rr.publish(msg)

def main(args=None):
    rcl.init(args=args)
    node = joy()
    rcl.spin(node)
    rcl.shutdown()

if __name__ == "__main__":
    main()



