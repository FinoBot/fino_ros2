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
        self.subscription_state

        self.subscription_target = self.create_subscription(
            DetectedPerson,
            '/detected_person',
            self.person_callback,
            10
        )
        self.subscription_target

        self.state_instruction_reply = self.create_publisher(String, '/state_instruction_reply', 10)
        
        self.command_client = self.create_client(ExecuteCommand, "execute_command")
        while not self.command_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('serial service not available, waiting again...')
        self.req = ExecuteCommand.Request()

        self.client_futures = []
        self.current_target = None
        self.current_state = None
        self.last_command = None
        self.lost_person_counter = 0
        self.get_logger().info("Movement controller node initialized")

    def state_callback(self, msg):
        self.current_state = msg.data
        if self.current_state == 'stop':
            self.send_command('kbalance')

    def handle_person_lost(self):
        if self.lost_person_counter > 25:
            self.send_command('kbalance')
            self.lost_person_counter = 0
            self.get_logger().info("Person lost, stopping movement")
            self.state_instruction_reply.publish(String(data='target_lost'))
        else:
            self.get_logger().info(f"Person lost, continuing movement {self.lost_person_counter}/25")
            self.lost_person_counter += 1

    def person_callback(self, msg):
        self.current_target = msg.position
        self.get_logger().info(f"current_state: {self.current_state} and current_target: {self.current_target}")

        if self.current_state == 'move_to_person':
            if msg.detected:
                distance = msg.position.z
                if distance < 1:
                    self.get_logger().info("Arrived at target, Ask to change to stand_by state")
                    self.state_instruction_reply.publish(String(data='target_reached'))
                else:
                    self.adjust_position(self.current_target)
            else:
                self.handle_person_lost()
        elif self.current_state == 'follow_person':
            if msg.detected:
                if distance < 1:
                    self.send_command('kbalance')
                else:
                    self.adjust_position(self.current_target)
            else:
                self.handle_person_lost()


    def adjust_position(self, position):
        x, z = position.x, position.z

        if abs(x) > 0.25:
            if x > 0:
                self.send_command('kwkR')  # Turn right
                self.get_logger().info("Adjusting position: turning right")
            else:
                self.send_command('kwkL')  # Turn left
                self.get_logger().info("Adjusting position: turning left")
        elif z > 1.5:
            self.send_command('kwkF')  # Move forward
            self.get_logger().info("Moving forward")
        else:
            self.send_command('kbalance')

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
