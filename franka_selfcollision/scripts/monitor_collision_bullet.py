import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from franka_msgs.srv import SelfCollisionBullet
import time
from functools import partial

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.latest_positions = None

        self.was_colliding = False
        
        self.create_timer(1, self.timer_callback)

        self.cli = self.create_client(SelfCollisionBullet, '/check_self_collision')
        while not self.cli.wait_for_service(timeout_sec=10):
            self.get_logger().info('Waiting for collision service...')
        self.get_logger().info('Collision Monitor Connected!')

    def joint_callback(self, msg: JointState):

        self.latest_positions = msg.position


    def timer_callback(self):
        if self.latest_positions is not None:
            self.send_check_request(self.latest_positions)
    
    def send_check_request(self, positions):
        req = SelfCollisionBullet.Request()
        req.joint_configuration = positions

        start_time = time.time()

        future = self.cli.call_async(req)
        future.add_done_callback(partial(self.response_callback, start_time))
    
    def response_callback(self, start_time, future):
        try:
            elapsed_time = time.time() - start_time
            response = future.result()
            is_colliding = response.collision

            self.get_logger().info(f"Check finished in {elapsed_time*1000:.2f} ms")


            if is_colliding and not self.was_colliding:
                self.get_logger().error(" COLLISION DETECTED! ")
                self.was_colliding = True

            elif not is_colliding and self.was_colliding:
                self.get_logger().info(" Collision Cleared :)")
                self.was_colliding = False

        except Exception as e:
            self.get_logger().error(f"Service failed : {e}")
        

def main():
    rclpy.init()
    node = CollisionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()