#!/usr/bin/env python3
"""
Simple manual arm control - Move joints by publishing to /joint_states
This is a workaround since ApplyJointEffort service is not available
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import sys, select, termios, tty

SPEEDS = {'1': (0.5, 0.5), '2': (1.0, 1.0), '3': (3.0, 2.0)}

msg = """
🤖 SIMPLE ARM CONTROL (Manual Joint Control) 🤖
------------------------------------------------
🚗 LAI XE: i, j, k, l, u, o (như cũ)

🦾 CANH TAY (Điều khiển thủ công):
   t/g : Vai (shoulder) LEN/XUONG
   y/h : Khuyu (elbow) GAP/DUOI  
   r/f : Kep (gripper) MO/DONG
   v   : Reset về vị trí ban đầu

CHE DO: 1=Cham | 2=Vua | 3=Nhanh
CTRL-C thoat

LUU Y: Cánh tay di chuyển TỰ DO (không có controller)
Bạn có thể kéo thủ công trong Gazebo
"""

class ManualArmControl(Node):
    def __init__(self):
        super().__init__('manual_arm_control')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_speed_mode = '2'
        
        # Arm positions (just for display, can't actually control without service)
        self.shoulder = 0.5
        self.elbow = -0.3
        self.gripper = 0.03
        
        print(msg)
        print("⚠️  WARNING: Arm control không hoạt động do thiếu Gazebo service")
        print("✅  Vehicle control hoạt động bình thường")
        print("💡  TIP: Bạn có thể kéo tay cánh trong Gazebo bằng chuột\n")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                
                linear = 0.0
                angular = 0.0
                target_v, target_w = SPEEDS[self.current_speed_mode]
                
                # Arm control (just updates internal state, can't move joints)
                if key == 't':
                    self.shoulder += 0.1
                    print(f"Vai: {self.shoulder:.2f} (chỉ hiển thị, không điều khiển)")
                elif key == 'g':
                    self.shoulder -= 0.1
                    print(f"Vai: {self.shoulder:.2f}")
                elif key == 'y':
                    self.elbow += 0.1
                    print(f"Khuỷu: {self.elbow:.2f}")
                elif key == 'h':
                    self.elbow -= 0.1
                    print(f"Khuỷu: {self.elbow:.2f}")
                elif key == 'r':
                    self.gripper += 0.01
                    print("Kẹp: MỞ")
                elif key == 'f':
                    self.gripper -= 0.01
                    print("Kẹp: ĐÓNG")
                elif key == 'v':
                    self.shoulder = 0.5
                    self.elbow = -0.3
                    self.gripper = 0.03
                    print("Reset")
                
                # Vehicle control (WORKS)
                if key == 'i': linear = target_v
                elif key == ',': linear = -target_v
                elif key == 'u': linear = target_v; angular = target_w
                elif key == 'o': linear = target_v; angular = -target_w
                elif key == 'j': angular = target_w
                elif key == 'l': angular = -target_w
                elif key == 'k': linear = 0.0; angular = 0.0
                elif key in ['1', '2', '3']:
                    self.current_speed_mode = key
                    continue
                elif key == '\x03':
                    break
                
                twist = Twist()
                twist.linear.x = float(linear)
                twist.angular.z = float(angular)
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ManualArmControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
