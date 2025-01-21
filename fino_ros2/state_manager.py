import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fino_ros2_msgs.srv import ExecuteCommand

AUDIO_COMMANDS_FOLLOW = "suis-moi"
AUDIO_COMMANDS_STOP = "stop"
AUDIO_COMMANDS_HI = ["coucou", "salut", "hey"]

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.state = 'initialization'
        self.target_position = None
        self.last_action_time = time.time()
        self.last_action = None

        # Subscription for the audio commands
        self.audio_commands = self.create_subscription(
            String,
            'audio_commands',
            self.audio_commands_callback,
            10
        )
        self.audio_commands

        self.state_instruction_reply = self.create_subscription(
            String,
            'state_instruction_reply',
            self.state_instruction_reply_callback,
            10
        )
        self.state_instruction_reply


        self.manual_state_change = self.create_subscription(
            String,
            'manual_state_change',
            self.manual_state_change_callback,
            10
        )
        self.manual_state_change

        # Publication des commandes d'état
        self.send_state_instruction = self.create_publisher(String, '/send_state_instruction', 10)

        self.command_client = self.create_client(ExecuteCommand, "execute_command")
        while not self.command_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('serial service not available, waiting again...')
        self.req = ExecuteCommand.Request()

        self.client_futures = []

        # Timer pour gérer les états
        self.timer = self.create_timer(0.5, self.update_state)

    def manual_state_change_callback(self, msg):
        self.change_state(msg.data.lower())

    def detection_callback(self, msg):
        # Mise à jour de la position cible à partir de la détection
        if msg.detected:
            self.target_position = msg.position
            self.get_logger().info(f"Received target position: x={self.target_position.x}, z={self.target_position.z}")
        else:
            self.target_position = None
            self.get_logger().info("Did not receive target")

    def audio_commands_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received audio command: {command}")

        if command == AUDIO_COMMANDS_FOLLOW:
            self.change_state('following')
        elif command == AUDIO_COMMANDS_STOP:
            self.change_state('stop')
        elif command in AUDIO_COMMANDS_HI and (self.state != 'search_interaction' or self.state != 'following'):
            self.change_state('hi')

    def state_instruction_reply_callback(self, msg):
        if self.state == 'search_interaction' and msg.data == 'target_reached':
            self.get_logger().info("Received target_reached")
            self.change_state('stand_by')
        elif self.state == 'search_interaction' and msg.data == 'target_lost':
            self.get_logger().info("Received target_lost from search_interaction")
        elif self.state == "following" and msg.data == 'target_lost':
            self.get_logger().info("Received target_lost from following")
            self.change_state('stand_by')

    def change_state(self, state):
        self.state = state
        self.get_logger().info(f"Changing state: {self.state}")
        self.publish_command(state)

    def update_state(self):
        now = time.time()

        if self.state == 'initialization':
            self.get_logger().info("Checking topics and services...")
            # Initialisation logic
            self.send_command('kbalance')
            time.sleep(1)
            self.change_state('following')

        elif self.state == 'hi':
            self.get_logger().info("Saying hi...")
            self.send_command('khi')
            time.sleep(5)
            self.change_state('stand_by')

        elif self.state == 'search_interaction':
            pass

        elif self.state == 'stop':
            time.sleep(1)
            self.change_state('stand_by')

        elif self.state == 'stand_by':
            self.send_command('ksit')
            if now - self.last_action_time > 120:  # 2 minutes
                self.get_logger().info("Switching to rest state")
                self.change_state('rest')

        elif self.state == 'rest':
            self.send_command('krest')
            if self.target_position:
                self.get_logger().info("New target after rest, switching to search_interaction state")
                self.change_state('search_interaction')
                self.send_command('kbalance')

        elif self.state == 'following':
            pass

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.send_state_instruction.publish(msg)

    def send_command(self, command):
        if command != self.last_action or command == 'khi':
            self.req.command = command
            self.client_futures.append(self.command_client.call_async(self.req))
            self.last_action = command if command != 'khi' else None
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
