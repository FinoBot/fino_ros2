import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PointStamped
import time
import uuid

LABELS = ['background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        self.camera_detections = self.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.detection_callback,
            10
        )
        self.camera_detections
        self.publisher = self.create_publisher(PointStamped, '/detected_person', 10)

        # Historical detections: {uuid: (initial detection time, current position)}
        self.detection_history = {}
        self.get_logger().info("Human detector node initialized")

    def detection_callback(self, msg):
        self.get_logger().info("Camera detection callback")
        current_time = time.time()
        new_detections = {}

        for detection in msg.detections:
            class_id = detection.results[0].hypothesis.class_id
            if LABELS[int(class_id)] == "person":
                self.get_logger().info("Human detected")
                position = detection.results[0].pose.pose.position
                self.get_logger().info(f"Position {position}")
                detection_id = self.get_detection_id(detection)

                # Update or add the detection
                if detection_id in self.detection_history:
                    new_detections[detection_id] = self.detection_history[detection_id]
                else:
                    new_detections[detection_id] = (current_time, position)

        # Update the history
        self.detection_history = new_detections
        self.get_logger().info(f"Detections: {self.detection_history}")

        # Find the closest and stable person
        closest_person = self.get_stable_closest_person(current_time)

        if closest_person:
            self.publish_human_position(closest_person)

    def get_detection_id(self, detection):
        """
        Create or retrieve a unique identifier for a detection.
        """
        position = detection.results[0].pose.pose.position
        for detection_id, data in self.detection_history.items():
            if self.is_same_detection(data[1], position):
                return detection_id
        return str(uuid.uuid4())

    def is_same_detection(self, old_position, new_position):
        """
        Determine if two positions are from the same detection.
        """
        distance_threshold = 1.5  # Adjust this threshold as needed
        distance = ((old_position.x - new_position.x) ** 2 +
                    (old_position.y - new_position.y) ** 2 +
                    (old_position.z - new_position.z) ** 2) ** 0.5
        return distance < distance_threshold

    def get_stable_closest_person(self, current_time):
        """
        Find the closest person
        """
        stable_persons = [
            (detection_id, data[1])  # (id_personne, position)
            for detection_id, data in self.detection_history.items()
            if current_time - data[0] >= 5
        ]

        if not stable_persons:
            return None

        # Find the closest person
        return min(stable_persons, key=lambda p: p[1].z)[1]

    def publish_human_position(self, position):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.point.x = position.x
        msg.point.y = position.y
        msg.point.z = position.z

        self.publisher.publish(msg)
        self.get_logger().info(f"Published closest stable person at position: {position}")

def main(args=None):
    rclpy.init(args=args)
    human_detector = HumanDetector()

    try:
        rclpy.spin(human_detector)
    except KeyboardInterrupt:
        pass
    finally:
        human_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()