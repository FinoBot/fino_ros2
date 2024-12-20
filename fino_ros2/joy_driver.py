import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from fino_ros2_msgs.srv import ExecuteCommand

skill_list = ["kbalance", "kwkF", "kwkR", "kwkL", "kbk", "kcalib", "khi", "ksit"]

class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_controler')

        self.command_client = self.create_client(ExecuteCommand, "execute_command")
        while not self.command_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('service not available, waiting again...')
        self.req = ExecuteCommand.Request()

        self.client_futures = []
        # self.dir = 'kbalance'
        # self.last_call = None
        # self.button_pressed = False
        # self.kbalance_sent = False
        self.previous_buttons = [0] * 4  # Assuming there are 4 buttons
        self.previous_axes = [0] * 2

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10
        )
        self.joy_subscription

        self.has_been_called = False

    def send_command(self, command):
            self.get_logger().info('calling service')
            self.req.command = command
            self.client_futures.append(self.command_client.call_async(self.req))

    def listener_callback(self, msg):
        #self.get_logger().info(f"data : {msg.buttons}")

        dir = None
        if not self.has_been_called:
            # Action to perform only on the first call
            dir = "kbalance"
            self.has_been_called = True

        button_commands = {
            0: "khi",
            1: "ksit",
            2: "krest",
            3: "kcalib"
        }

        axis_commands = {
            (1, 1.0): "kwkF",
            (1, -1.0): "kbk",
            (0, 1.0): "kwkL",
            (0, -1.0): "kwkR"
        }

        # Check button presses
        for i, command in button_commands.items():
            if msg.buttons[i] == 1 and self.previous_buttons[i] == 0:
                dir = command
                break
        else:
            # Check axes for movement
            for (axis, value), command in axis_commands.items():
                if msg.axes[axis] == value and self.previous_axes[axis] != value:
                    dir = command
                    break
                else:
                    if msg.axes[axis] == 0 and (self.previous_axes[axis] == 1.0 or self.previous_axes[axis] == -1.0):
                        dir = "kbalance"
                        break
            
        if dir is not None:
            self.get_logger().info(f"Sending command : {dir}")
            self.send_command(dir)
        
        # Update previous states
        self.previous_buttons = msg.buttons[:4]
        self.previous_axes = msg.axes[:2]

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    print("received service result: {}".format(res))
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures


def main(args=None):
    rclpy.init(args=args)

    joy_controler = JoyControl()
    #joy_controler.spin()
    rclpy.spin(joy_controler)
    
    joy_controler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()