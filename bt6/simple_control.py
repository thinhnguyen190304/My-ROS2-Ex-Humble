import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- CAI DAT TOC DO (3 CAP DO) ---
# Format: (Toc do di thang, Toc do xoay)
SPEEDS = {
    '1': (0.5, 0.5),   # Cap 1: Cham (Rua)
    '2': (1.0, 1.0),   # Cap 2: Vua (Tho)
    '3': (3.0, 2.0)    # Cap 3: SIUE TOC DO (Pha vo gioi han)
}

msg = """
DIEU KHIEN ROBOT - PHIEN BAN PRO
---------------------------
DI CHUYEN THANG:
   i : Di thang
   , : Di lui

CUA (VUA DI VUA RE - Giong lai xe):
   u : Re Cung Tron Trai (Di thang + Trai)
   o : Re Cung Tron Phai (Di thang + Phai)

QUAY TAI CHO:
   j : Quay sang trai
   l : Quay sang phai

   k : DUNG LAI

CHON TOC DO:
   1 : Cham
   2 : Vua
   3 : SIUE TOC DO
"""

class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_speed_mode = '2'
        print(msg)
        self.print_speed()

    def print_speed(self):
        v, w = SPEEDS[self.current_speed_mode]
        modes = {'1': "CHAM", '2': "VUA", '3': "SIEU TOC DO"}
        print(f"--> CHE DO: {modes[self.current_speed_mode]} (Max Speed: {v})")

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

                # --- LOGIC DIEU KHIEN ---
                if key == 'i':
                    linear = target_v       # Di thang
                elif key == ',':
                    linear = -target_v      # Di lui
                elif key == 'u':            # CUA TRAI
                    linear = target_v
                    angular = target_w
                elif key == 'o':            # CUA PHAI
                    linear = target_v
                    angular = -target_w
                elif key == 'j':            # QUAY TRAI TAI CHO
                    linear = 0.0
                    angular = target_w
                elif key == 'l':            # QUAY PHAI TAI CHO
                    linear = 0.0
                    angular = -target_w
                elif key == 'k':            # DUNG
                    linear = 0.0
                    angular = 0.0
                
                elif key in ['1', '2', '3']:
                    self.current_speed_mode = key
                    self.print_speed()
                    continue
                
                elif key == '\x03':
                    break
                
                twist = Twist()
                twist.linear.x = float(linear)
                twist.angular.z = float(angular)
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.angular.z = 0.0
            self.publisher_.publish(twist)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = SimpleTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
