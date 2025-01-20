# target_node.py
import rclpy
from rclpy.node import Node
from fino_ros2_msgs.msg import DetectedPerson
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String


class Following(LifecycleNode):
    def __init__(self):
        super().__init__('Following')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Following Node...')
        self.subscription = self.create_subscription(
            DetectedPerson,
            '/detected_person',
            self.detection_callback,
            10
        )
        self.subscription_state = self.create_subscription(
            String,
            '/state_command',
            self.state_callback,
            10
        )

        return TransitionCallbackReturn.SUCCESS

    def detection_callback(self, msg):
        now = self.get_clock().now().nanoseconds
        if msg.detected:
            self.get_logger().info(f"Person detected at position: {msg.position}")
            distance = self.target_position.z
            if distance <= 1.5:
                self.get_logger().info("Arrived at target, switching to stand_by state")
                self.publish_command('stop')
        else:
            self.get_logger().info("Lost target, searching for new target")
            self.send_command('ksit')

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Following Node...')
        if self.subscription is not None:
            self.subscription  # This line is not necessary, just ensure the subscription is created in on_configure
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Following Node...')
        if self.subscription is not None:
            self.subscription  # This line is not necessary, just ensure the subscription is managed properly
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Following Node...')
        self.destroy_subscription(self.subscription)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down Following Node...')
        return TransitionCallbackReturn.SUCCESS



def main(args=None):
    rclpy.init(args=args)
    target_node = Following()
    rclpy.spin(target_node.get_node_base_interface())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
