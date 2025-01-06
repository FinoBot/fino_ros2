import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray

LABELS = ['background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']

class Interaction(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('started listening to /oak/nn/spatial_detections')


    def listener_callback(self, msg):
        if len(msg.detections) > 0:
            for index, detection in enumerate(msg.detections):
                self.get_logger().info(f'detection : {index}, class: {LABELS[int(detection.results[0].hypothesis.class_id)]} - pose : {detection.results[0].pose.pose.position}')


def main(args=None):
    rclpy.init(args=args)

    interaction = Interaction()

    rclpy.spin(interaction)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    interaction.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()