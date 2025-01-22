import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fino_ros2_msgs.msg import DetectedPerson
from fino_ros2_msgs.srv import ExecuteCommand
import time

class DisplacementController(Node):
    def __init__(self):
        super().__init__('displacement_controller')
        self.instruction_subscription = self.create_subscription(
            String,
            '/send_state_instruction',
            self.instruction_callback,
            10
        )
        self.instruction_subscription

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
        self.lost_person_ts = None
        self.lost_person_following_ts = None
        self.lost_person_countdown = 0
        self.get_logger().info("Displacement controller node initialized")

    def instruction_callback(self, msg):
        self.current_state = msg.data
        if self.current_state == 'stop':
            self.send_command('kbalance')

    def handle_person_lost(self):
        lost_person_following_delay = 3
        lost_person_delay = 25

        if self.lost_person_ts is None:
            self.lost_person_ts = time.time()
        if self.lost_person_following_ts is None:
            self.lost_person_following_ts = time.time()

        if time.time() - self.lost_person_following_ts > lost_person_following_delay:
            self.send_command('kbalance')
        elif time.time() - self.lost_person_ts > lost_person_delay:
            self.get_logger().info("Person lost, stopping movement")
            self.state_instruction_reply.publish(String(data='target_lost'))
            self.lost_person_ts = None
        else:
            if self.lost_person_countdown != int(time.time() - self.lost_person_following_ts):
                self.get_logger().info(f"Person lost, waiting  {abs(int(time.time() - self.lost_person_following_ts - lost_person_following_delay+1))}s to recover")
            self.lost_person_countdown = int(time.time() - self.lost_person_following_ts)

    def person_callback(self, msg):
        self.current_target = msg.position

        #self.get_logger().info(f"current_state: {self.current_state} and current_target: {self.current_target}")

        if self.current_state == 'search_interaction':
            if msg.detected:
                self.lost_person_ts = None
                self.lost_person_following_ts = None
                if msg.position.z < 1.1:
                    self.get_logger().info("Arrived at target, Ask to change to stand_by state")
                    self.state_instruction_reply.publish(String(data='target_reached'))
                else:
                    self.adjust_position(self.current_target)
            else:
                self.handle_person_lost()
        elif self.current_state == 'following':
            if msg.detected:
                self.lost_person_ts = None
                self.lost_person_following_ts = None
                if msg.position.z < 1.1:
                    self.send_command('kbalance')
                    self.get_logger().info("displacement controller determined that the target is close enough")
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
        elif z > 1.1:
            self.send_command('kwkF')  # Move forward
            self.get_logger().info("Moving forward")

    def send_command(self, command):
        if command == self.last_command:
            return
        self.req.command = command
        self.client_futures.append(self.command_client.call_async(self.req))
        self.last_command = command

def main(args=None):
    rclpy.init(args=args)
    displacement_controller = DisplacementController()

    try:
        rclpy.spin(displacement_controller)
    except KeyboardInterrupt:
        pass
    finally:
        displacement_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
