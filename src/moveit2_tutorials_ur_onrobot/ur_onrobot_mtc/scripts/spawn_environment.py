#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class EnvironmentSpawner(Node):
    def __init__(self):
        super().__init__('environment_spawner')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(CollisionObject, '/collision_object', qos_profile)
        self.timer = self.create_timer(0.5, self.spawn_scene)

    def spawn_scene(self):
        # 1. Spawn Table (Mặt trên bàn hạ xuống z = -0.003m)
        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'table'
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.0, 1.0, 0.1])
        t_pose = Pose()
        t_pose.position.z = -0.053
        t_pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(t_pose)
        table.operation = CollisionObject.ADD
        self.publisher.publish(table)

        # 2. Spawn Left Object (Nằm chuẩn trên mặt bàn mới)
        left_obj = CollisionObject()
        left_obj.header.frame_id = 'world'
        left_obj.id = 'left_object'
        cyl = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.1, 0.02])
        l_pose = Pose()
        l_pose.position.x, l_pose.position.y, l_pose.position.z = 0.3, 0.2, 0.02
        l_pose.orientation.x, l_pose.orientation.w = 0.707, 0.707
        left_obj.primitives.append(cyl)
        left_obj.primitive_poses.append(l_pose)
        left_obj.operation = CollisionObject.ADD
        self.publisher.publish(left_obj)

        # 3. Spawn Right Object (Nằm chuẩn trên mặt bàn mới)
        right_obj = CollisionObject()
        right_obj.header.frame_id = 'world'
        right_obj.id = 'right_object'
        r_pose = Pose()
        r_pose.position.x, r_pose.position.y, r_pose.position.z = 0.3, -0.2, 0.02
        r_pose.orientation.x, r_pose.orientation.w = 0.707, 0.707
        right_obj.primitives.append(cyl)
        right_obj.primitive_poses.append(r_pose)
        right_obj.operation = CollisionObject.ADD
        self.publisher.publish(right_obj)

        self.get_logger().info('Đã cập nhật tọa độ Bàn và Vật thể mới tránh va chạm đế!')
        self.timer.cancel()

def main():
    rclpy.init()
    node = EnvironmentSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()