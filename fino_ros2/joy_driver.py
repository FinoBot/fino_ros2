import rclpy
from rclpy.node import Node
from time import sleep
from time import time

from sensor_msgs.msg import Joy
from fino_ros2_msgs.srv import ExecuteCommand

class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_controler')

        self.command_client = self.create_client(ExecuteCommand, "execute_command")
        while not self.command_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('service not available, waiting again...')
        self.req = ExecuteCommand.Request()

        self.client_futures = []

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10
        )
        self.joy_subscription

    def send_command(self, command):
            self.get_logger().info('calling service')
            self.req.command = command
            self.client_futures.append(self.command_client.call_async(self.req))

    def listener_callback(self, msg):
        self.get_logger().info(f"data : {msg.buttons}")

        # Balance
        if msg.buttons[0] == 1:
            self.send_command("kbalance")

        # Button2
        if msg.buttons[1] == 1:
            pass

        # Button3
        if msg.buttons[2] == 1:
            pass

        # Button4
        if msg.buttons[3] == 1:
            self.send_command("kcalib")

        # Avancer
        if msg.axes[1] == 1.0:
            self.send_command("kwkF")

        # Reculer
        if msg.axes[1] == -1.0:
            self.send_command("kbk")

        # Tourner à gauche
        if msg.axes[0] == 1.0:
            pass

        # Tourner à droite
        if msg.axes[0] == -1.0:
            pass

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