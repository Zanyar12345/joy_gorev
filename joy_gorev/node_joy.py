#!/usr/bin/env python3
import rclpy as rcl
from rclpy.node import Node
from sensor_msgs.msg import Joy
from example_interfaces.msg import Int32
import math

#63 90 180 270 sol,ileri + sağ,geri -

class joy(Node):
    def __init__(self):
        super().__init__("Publisher")
        self.create_subscription(Joy,"joy",self.msg,10)
        self.angle=self.create_publisher(Int32,"wheel_front_left_angle",10)
        self.speed=self.create_publisher(Int32,"wheel_front_left_speed",10)
    def msg(self,data1:Joy):
        msg1=Int32()
        msg1.data=int(data1.axes[1]*63)
        self.speed.publish(msg1)
        msg2=Int32()
        msg2.data=int((math.degrees(math.atan2(data1.axes[1],-data1.axes[0]))+90)%360)
        self.angle.publish(msg2)

def main(args=None):
    rcl.init(args=args)
    node=joy()
    rcl.spin(node)
    rcl.shutdown()

if __name__=="__main__":
    main()