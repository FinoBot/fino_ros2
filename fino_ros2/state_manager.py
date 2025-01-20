import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fino_ros2_msgs.msg import DetectedPerson
from fino_ros2_msgs.srv import ExecuteCommand
import time
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition


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
            DetectedPerson,
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

        # Services pour les noeuds externes
    #     self.change_state_following_minigame = self.create_client(ChangeState, '/<node_name>/change_state')
    #     self.get_state_following_minigame = self.create_client(GetState, '/<node_name>/get_state')

    # def get_current_external_node_state(self, get_state_following):
    #     request = GetState.Request()
    #     future = self.get_state_following.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         state_id = future.result().current_state.id
    #         state_label = future.result().current_state.label
    #         return f'{state_label} (ID: {state_id})'
    #     else:
    #         self.get_logger().error('Failed to get current state.')
    #         return 'Unknown'

    # def activate_external_node_state(self, transition_id):
    #     request = ChangeState.Request()
    #     request.transition.id = transition_id

    #     future = self.change_state_following.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None and future.result().success:
    #         self.get_logger().info(f'Successfully performed transition ID {transition_id}.')
    #     else:
    #         self.get_logger().error(f'Failed to perform transition ID {transition_id}.')

    def detection_callback(self, msg):
        # Mise à jour de la position cible à partir de la détection
        if msg.detected:
            self.target_position = msg.position
            self.last_seen_time = time.time()
            self.get_logger().info(f"Received target position: x={self.target_position.x}, z={self.target_position.z}")
        else:
            self.target_position = None
            self.get_logger().info("Did not receive target")

    def change_state(self, state):
        self.state = state
        self.publish_command(state)


    def update_state(self):
        now = time.time()
        self.get_logger().info(f"Current state: {self.state}")

        if self.state == 'initialization':
            self.get_logger().info("Checking topics and services...")
            # Initialisation logic
            self.send_command('kbalance')
            time.sleep(1)
            self.change_state('following')

        elif self.state == 'search_interaction':
            self.get_logger().info("Searching for interaction...")
            if self.target_position and self.target_position.z > 1.5:
                self.get_logger().info("Switching to move_to_person state")
                self.change_state('move_to_person')

        elif self.state == 'move_to_person':
            if self.target_position:
                distance = self.target_position.z
                if distance <= 1.5:
                    self.get_logger().info("Arrived at target, switching to stand_by state")
                    self.change_state('stand_by')
                elif now - self.last_seen_time > 1:
                    self.get_logger().info("Lost target, switching to search_interaction state")
                    self.change_state('search_interaction')
                    self.send_command('kbalance')

        elif self.state == 'stand_by':
            self.send_command('ksit')
            if now - self.last_action_time > 120:  # 2 minutes
                self.get_logger().info("Switching to rest state")
                self.change_state('rest')

        elif self.state == 'rest':
            if self.target_position:
                self.get_logger().info("New target after rest, switching to search_interaction state")
                self.change_state('search_interaction')
                self.send_command('kbalance')

        elif self.state == 'following':
            pass

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Command sent: {command}")

    def send_command(self, command):
        if command != self.last_action:
            self.req.command = command
            self.client_futures.append(self.command_client.call_async(self.req))
            self.last_action = command
            self.last_action_time = time.time()

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
