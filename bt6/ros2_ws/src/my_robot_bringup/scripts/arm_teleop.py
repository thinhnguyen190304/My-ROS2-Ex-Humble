#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import sys, select, termios, tty

msg = """
========================================================
       DIEU KHIEN CANH TAY BANG BAN PHIM (Stable)
========================================================
KHỚP VAI (Shoulder):
    w : Lên
    s : Xuống

KHỚP KHUYỬ (Elbow):
    a : Trái (Co)
    d : Phải (Duỗi)

BÀN GẮP (Gripper):
    o : MỞ 👐
    c : ĐÓNG ✊

LỆNH GẮP LOGIC:
    g : GẮP (Attach)
    h : NHẢ (Detach)

Dừng lại: [Phím bất kỳ khác]
Thoát: [Ctrl+C]
========================================================
"""

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Current positions
        self.shoulder = 0.0
        self.elbow = 0.0
        
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Step sizes
        self.arm_step = 0.05
        self.gripper_step = 0.01

    def joint_state_callback(self, msg):
        try:
            s_idx = msg.name.index('shoulder_joint')
            e_idx = msg.name.index('elbow_joint')
            self.shoulder = msg.position[s_idx]
            self.elbow = msg.position[e_idx]
        except ValueError:
            pass

    def send_arm_cmd(self, s_delta, e_delta):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.shoulder + s_delta, self.elbow + e_delta]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 0.2s
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def send_gripper_cmd(self, pos):
        traj = JointTrajectory()
        traj.joint_names = ['left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [pos, pos]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000
        traj.points.append(point)
        self.gripper_pub.publish(traj)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArmTeleop()
    
    print(msg)
    
    try:
        while True:
            key = get_key(settings)
            if key == 'w':
                node.send_arm_cmd(node.arm_step, 0.0)
            elif key == 's':
                node.send_arm_cmd(-node.arm_step, 0.0)
            elif key == 'a':
                node.send_arm_cmd(0.0, node.arm_step)
            elif key == 'd':
                node.send_arm_cmd(0.0, -node.arm_step)
            elif key == 'o':
                node.send_gripper_cmd(0.25)
            elif key == 'c':
                node.send_gripper_cmd(0.0)
            elif key == 'g':
                import subprocess
                subprocess.run(["python3", "/mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/src/my_robot_bringup/scripts/grasp_logic.py", "attach"])
            elif key == 'h':
                import subprocess
                subprocess.run(["python3", "/mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/src/my_robot_bringup/scripts/grasp_logic.py", "detach"])
            elif key == '\x03': # Ctrl+C
                break
            
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
