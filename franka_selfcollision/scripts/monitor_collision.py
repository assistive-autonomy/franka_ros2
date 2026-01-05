#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from functools import partial
import time

from franka_msgs.srv import SelfCollision

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        self.target_joint_order = [
            # left arm
            'left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3', 'left_fr3_joint4',
            'left_fr3_joint5', 'left_fr3_joint6', 'left_fr3_joint7',
            # right arm
            'right_fr3_joint1', 'right_fr3_joint2', 'right_fr3_joint3', 'right_fr3_joint4',
            'right_fr3_joint5', 'right_fr3_joint6', 'right_fr3_joint7'
        ]
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/collision_marker', 10)

        self.latest_positions = None

        self.was_colliding = False

        self.publish_marker(True)

        self.create_timer(0.033, self.timer_callback)

        self.cli = self.create_client(SelfCollision, '/check_self_collision')
        while not self.cli.wait_for_service(timeout_sec=10):
            self.get_logger().info('Waiting for collision service...')
        self.get_logger().info('Collision Monitor Connected!')

    def joint_callback(self, msg: JointState):
        state_dict = dict(zip(msg.name, msg.position))

        current_positions = []

        try:
            for name in self.target_joint_order:
                current_positions.append(state_dict[name])

            self.latest_positions = current_positions

        except KeyError as e:
            self.get_logger().info(e.what())

    def timer_callback(self):
        if self.latest_positions is not None:
            self.send_check_request(self.latest_positions)

    def send_check_request(self, positions):
        req = SelfCollision.Request()
        req.joint_configuration = positions

        start_time = time.time()

        future = self.cli.call_async(req)
        future.add_done_callback(partial(self.response_callback, start_time))

    def response_callback(self, start_time, future):
        try:
            elapsed_time = time.time() - start_time
            response = future.result()
            is_colliding = response.collision

            self.publish_marker(is_colliding)

            print_time = False

            if print_time:
                self.get_logger().info(f'Check finished in {elapsed_time*1000:.2f} ms')

            if is_colliding and not self.was_colliding:
                self.get_logger().error('⚠️  COLLISION DETECTED!')
                self.was_colliding = True

            elif not is_colliding and self.was_colliding:
                self.get_logger().info('✅ Collision Cleared:)')
                self.was_colliding = False

        except Exception as e:
            self.get_logger().error(f'Service failed : {e}')

    def publish_marker(self, is_colliding):
        marker = Marker()
        marker.header.frame_id = 'base'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'collision_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.15

        marker.color.a = 1.0

        if is_colliding:
            marker.text = ' COLLISION! '
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        else:
            marker.text = ' NO COLLISION '
            marker.color.r = 0.2
            marker.color.g = 1.0
            marker.color.b = 0.2

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = CollisionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
