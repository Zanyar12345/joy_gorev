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
        delta = 90*data1.axes[0]+90 # önce 0 180 yapıp karışıklığı engelledim


        R = abs((self.length/2) / math.tan(math.radians(delta))) # merkezdenin noktaya uzaklığı

        if steer > 0:
                # FL_rad =  math.atan2((R + self.width / 2) , (self.length / 2))
                # FR_rad =  math.atan2(abs(R - self.width / 2) , (self.length / 2))
                RL_rad =  math.atan2((R + self.width / 2) , (self.length / 2))
                RR_rad =  math.atan2(abs(R - self.width / 2) , (self.length / 2))

                R_fl = ((R - self.width / 2)**2+(self.length / 2)**2)**(1/2) # uzaklıklar
                R_fr = ((R + self.width / 2)**2+(self.length / 2)**2)**(1/2)

                RL = (math.degrees(RL_rad)) # üst teker ile alt teker toplamı 180. küçükten büyüğü çıkardım çünkü diğeri geniş açı olacak
                RR = (math.degrees(RR_rad))
                FL = 180-RL
                FR = 180-RR

                FL = 270-(math.degrees(FL_rad))
                FR = 270-(math.degrees(FR_rad))
                RL = 270-(math.degrees(RL_rad)) # 270 90 arası olduğu için 270ten çıkardım
                RR = 270-(math.degrees(RR_rad))
        else:
                
                FL_rad =  math.atan2((R + self.width / 2) , (self.length / 2))
                FR_rad =  math.atan2(abs(R - self.width / 2) , (self.length / 2)) 
                # RL_rad =  math.atan2((R + self.width / 2) , (self.length / 2))
                # RR_rad =  math.atan2(abs(R - self.width / 2) , (self.length / 2))

                R_fl = ((R + self.width / 2)**2+(self.length / 2)**2)**(1/2) # yarıçaplar (alt üst eşit o yüzden yanları hesaplasak yeterli)
                R_fr = ((R - self.width / 2)**2+(self.length / 2)**2)**(1/2)

                FL = (math.degrees(FL_rad))
                FR = (math.degrees(FR_rad))
                RL = 180-FL
                RR = 180-FR

                FL = 270-(math.degrees(FL_rad))
                FR = 270-(math.degrees(FR_rad))
                RL = 270-(math.degrees(RL_rad))
                RR = 270-(math.degrees(RR_rad))

        if R_fl>=R_fr:
                v_fl = base_speed # Yarıçap büyük hız büyük
                v_fr = base_speed*(R_fr / R_fl) # Yarıçapı büyüğe göre oranlama
        else:
                v_fl = base_speed*(R_fl / R_fr) 
                v_fr = base_speed

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



