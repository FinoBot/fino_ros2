import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fino_ros2_msgs.msg import DetectedPerson
from fino_ros2_msgs.srv import ExecuteCommand

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.subscription_state = self.create_subscription(
            String,
            '/state_command',
            self.state_callback,
            10
        )
        self.subscription_target = self.create_subscription(
            DetectedPerson,
            '/detected_person',
            self.person_callback,
            10
        )
        
        self.command_client = self.create_client(ExecuteCommand, "execute_command")
        while not self.command_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('serial service not available, waiting again...')
        self.req = ExecuteCommand.Request()

        self.client_futures = []
        self.current_target = None
        self.current_state = None
        self.last_command = None
        self.get_logger().info("Movement controller node initialized")

    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f"State command received: {self.current_state}")
        if self.current_state == 'stop':
            self.send_command('kbalance')

    def person_callback(self, msg):
        self.current_target = msg.position
        self.get_logger().info(f"current_state: {self.current_state} and current_target: {self.current_target}")
        if self.current_state == 'move_to_person' or self.current_state == 'following':
            if msg.detected:
                self.adjust_position(self.current_target)

    def adjust_position(self, position):
        x, z = position.x, position.z

        if abs(x) > 0.25:
            if x > 0:
                self.send_command('kwkR')  # Turn right
                self.get_logger().info("Adjusting position: moving right")
            else:
                self.send_command('kwkL')  # Turn left
                self.get_logger().info("Adjusting position: moving left")
        elif z > 1.5:
            self.send_command('kwkF')  # Move forward
            self.get_logger().info("Moving forward")
        else:
            self.send_command('kbalance')
            self.get_logger().info("entering else, doing kbalance")

    def send_command(self, command):
        if command == self.last_command:
            return
        self.req.command = command
        self.client_futures.append(self.command_client.call_async(self.req))
        self.last_command = command

def main(args=None):
    rclpy.init(args=args)
    movement_controller = MovementController()

    try:
        rclpy.spin(movement_controller)
    except KeyboardInterrupt:
        pass
    finally:
        movement_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
