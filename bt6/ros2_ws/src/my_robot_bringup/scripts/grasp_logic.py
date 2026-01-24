#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class GraspLogic(Node):
    def __init__(self):
        super().__init__('grasp_logic')
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt /apply_planning_scene service...')
        
    def attach_box(self):
        # Create a virtual box to attach (simplified demo logic)
        # In a real scenario, you'd find the nearest object in the scene
        ps = PlanningScene()
        ps.is_diff = True
        
        aco = AttachedCollisionObject()
        aco.link_name = "gripper_base"
        
        # Define a small box that will be "locked" to the gripper
        obj = CollisionObject()
        obj.header.frame_id = "gripper_base"
        obj.id = "grasped_object"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]
        
        pose = Pose()
        pose.position.z = 0.05 # Position relative to gripper_base
        pose.orientation.w = 1.0
        
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        
        aco.object = obj
        ps.robot_state.attached_collision_objects.append(aco)
        
        request = ApplyPlanningScene.Request()
        request.scene = ps
        self.client.call_async(request)
        print("✅ ATTACHED: Vật thể đã được khóa vào tay gắp!")

    def detach_box(self):
        ps = PlanningScene()
        ps.is_diff = True
        
        aco = AttachedCollisionObject()
        aco.link_name = "gripper_base"
        obj = CollisionObject()
        obj.id = "grasped_object"
        obj.operation = CollisionObject.REMOVE
        aco.object = obj
        
        ps.robot_state.attached_collision_objects.append(aco)
        
        request = ApplyPlanningScene.Request()
        request.scene = ps
        self.client.call_async(request)
        print("❌ DETACHED: Đã nhả vật thể!")

def main():
    rclpy.init()
    node = GraspLogic()
    
    if len(sys.argv) < 2:
        print("Usage: grasp_logic.py [attach|detach]")
        return

    cmd = sys.argv[1]
    if cmd == "attach":
        node.attach_box()
    elif cmd == "detach":
        node.detach_box()
    
    # Wait a bit for the async call to finish
    import time
    time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
