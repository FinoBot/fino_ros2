import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from fino_ros2_msgs.srv import ExecuteCommand
import time

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.state = 'initialization'
        self.target_position = None
        self.last_seen_time = None
        self.last_action_time = time.time()
        self.last_action = None

        # Abonnement à la détection des humains
        self.detection_sub = self.create_subscription(
            PointStamped,
            '/detected_person',
            self.detection_callback,
            10
        )

        # Publication des commandes d'état
        self.command_pub = self.create_publisher(String, '/state_command', 10)

        self.command_client = self.create_client(ExecuteCommand, "execute_command")
        while not self.command_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('serial service not available, waiting again...')
        self.req = ExecuteCommand.Request()

        self.client_futures = []
        # Timer pour gérer les états
        self.timer = self.create_timer(0.5, self.update_state)

    def detection_callback(self, msg):
        # Mise à jour de la position cible à partir de la détection

        self.target_position = msg.point
        self.last_seen_time = time.time()
        self.get_logger().info(f"Received target position: x={self.target_position.x}, z={self.target_position.z}")

    def update_state(self):
        now = time.time()

        if self.state == 'initialization':
            self.get_logger().info("Checking topics and services...")
            # Initialisation logic
            self.send_command('kbalance')
            time.sleep(1)
            self.state = 'search_interaction'

        elif self.state == 'search_interaction':
            self.get_logger().info("Searching for interaction...")
            # if self.last_action != 'ksit':
            #     self.send_command('ksit')
            #     self.last_action = 'ksit'
            if self.target_position:
                self.state = 'move_to_person'
                self.get_logger().info("Switching to move_to_person state")
                self.publish_command('move_to_person')

        elif self.state == 'move_to_person':
            if self.target_position:
                distance = self.target_position.z
                if distance <= 1.5:
                    self.state = 'stand_by'
                    self.get_logger().info("Arrived at target, switching to stand_by state")
                    self.publish_command('stop')
                elif now - self.last_seen_time > 1:
                    self.state = 'search_interaction'
                    self.get_logger().info("Lost target, switching to search_interaction state")
                    self.publish_command('search_interaction')
                    self.send_command('kbalance')

        elif self.state == 'stand_by':
            if now - self.last_action_time > 300:  # 5 minutes
                self.state = 'rest'
                self.get_logger().info("Switching to rest state")
                self.publish_command('rest')

        elif self.state == 'rest':
            self.get_logger().info("In rest state, conserving energy.")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Command sent: {command}")

    def send_command(self, command):
        self.req.command = command
        self.client_futures.append(self.command_client.call_async(self.req))

def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()

    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
