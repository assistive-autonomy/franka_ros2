import rclpy
from rclpy.node import Node
from franka_msgs.srv import SelfCollision

class CollisionClient(Node):
    def __init__(self):
        super().__init__('collision_tester')
        self.cli = self.create_client(SelfCollision, '/check_self_collision')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service Found!')

    def send_request(self, joints):
        req = SelfCollision.Request()
        req.joint_configuration = joints

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
def main():
    rclpy.init()
    client = CollisionClient()

    q_safe = [
         0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785,  # Arm 1
         0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785   # Arm 2
    ]

    print("\n Safe configuration:")
    result = client.send_request(q_safe)
    print(f"Result : {result.collision}")

    q_collision = [
         -0.45, 0.77, 0.12, -1.38, 0.0, 2.4, 0.52,
        0.26, 0.57, 0.03, -1.44, -0.38, 2.62, 1.34 
    ]

    print("\n Collision configuration:")
    result = client.send_request(joints=q_collision)
    print(f"Result : {result.collision}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()