from .SerialCommunication import SerialCommunication

import rclpy
from rclpy.node import Node
import time
import struct

from fino_ros2_msgs.srv import ExecuteCommand


class Driver(Node):

    def __init__(self):
        super().__init__('finobot_driver')
        self.srv = self.create_service(ExecuteCommand, 'execute_command', self.send_serial_command)
        self.serial_comm = SerialCommunication('/dev/ttyS0')
        self.serial_comm.open_connection()

    def send_serial_command(self, request, response):
        self.get_logger().info('Incoming serial command request')

        self.wrapper([request.command, 1])

        response.success = True

        return response
    
    def wrapper(self, task):  # Structure is [token, var=[], time]
        self.get_logger().info("Task : " + str(task))
        if len(task) == 2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0], int):
            self.serialWriteNumToByte(task[0], task[1])
        else:
            self.serialWriteByte(task[1])
        time.sleep(task[-1])
        
    def serialWriteNumToByte(self, token, var=[]):  # Only to be used for c m u b i l o within Python
        # print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
        if token == 'l' or token == 'i':
            var = list(map(lambda x: int(x), var))
            instrStr = token + struct.pack('b' * len(var), *var) + '~'
        elif token == 'c' or token == 'm' or token == 'u' or token == 'b':
            instrStr = token + str(var[0]) + " " + str(var[1]) + '\n'
        self.get_logger().info("!!!!" + instrStr)
        self.serial_comm.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        print(f"Token : {token}")
        if (token == 'c' or token == 'm' or token == 'b' or token == 'u') and len(var) >= 2:
            instrStr = ""
            for element in var:
                instrStr = instrStr + element + " "
        elif token == 'l' or token == 'i':
            if (len(var[0]) > 1):
                var.insert(1, var[0][1:])
            var[1:] = list(map(lambda x: int(x), var[1:]))
            instrStr = token + struct.pack('b' * len(var[1:]), *var[1:]) + '~'
        elif token == 'w' or token == 'k':
            print("inside w or k")
            print(var)
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        self.get_logger().info("!!!!!!! " + instrStr)
        self.serial_comm.write(instrStr.encode())


def main():
    rclpy.init()

    driver = Driver()

    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()