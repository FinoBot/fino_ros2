import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from fino_ros2_msgs.msg import DetectedPerson
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
        self.publisher = self.create_publisher(DetectedPerson, '/detected_person', 10)

        # Historical detections: {uuid: (initial detection time, current position)}
        self.detection_history = {}
        self.get_logger().info("Human detector node initialized")

    def detection_callback(self, msg):
        current_time = time.time()
        new_detections = {}

        for detection in msg.detections:
            class_id = detection.results[0].hypothesis.class_id
            if LABELS[int(class_id)] == "person":
                position = detection.results[0].pose.pose.position
                self.get_logger().info(f"Position {position}")
                detection_id = self.get_detection_id(detection)

                # Update or add the detection
                if detection_id in self.detection_history:
                    new_detections[detection_id] = self.detection_history[detection_id]
                    new_detections[detection_id]["time"] = self.detection_history[detection_id]["time"]
                    new_detections[detection_id]["position"] = position
                    #self.get_logger().info(f"Updated position of person to follow:  x={round(position.x,2)}, z={round(position.z,2)}")
                else:
                    new_detections[detection_id] = {}
                    new_detections[detection_id]["time"] = current_time
                    new_detections[detection_id]["position"] = position
                    self.get_logger().info(f"Added new person to follow:  x={round(position.x,2)}, z={round(position.z,2)}")

        # Update the history
        if new_detections:
            #self.get_logger().info(f"New detections : {new_detections}")
            self.detection_history = new_detections
            # Find the closest and stable person
            response_to_send = self.get_stable_closest_person(current_time)
        else:
            response_to_send = None
            #self.get_logger().info("No new detections")


        self.publish_human_position(response_to_send)

    def get_detection_id(self, detection):
        """
        Create or retrieve a unique identifier for a detection.
        """
        position = detection.results[0].pose.pose.position
        for detection_id, data in self.detection_history.items():
            if self.is_same_detection(data["position"], position):
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
            (detection_id, data["position"])  # (id_personne, position)
            for detection_id, data in self.detection_history.items()
            if current_time - data["time"] >= 5
        ]

        if not stable_persons:
            return None

        # Find the closest person
        return min(stable_persons, key=lambda p: p[1].z)[1]

    def publish_human_position(self, position):
        msg = DetectedPerson()
        if position:
            msg.detected = True
            msg.position.x = position.x
            msg.position.y = position.y
            msg.position.z = position.z
            #self.get_logger().info(f"Published closest stable person at position:  x={position.x}, z={position.z}")
        else:
            msg.detected = False
            #self.get_logger().info("No stable person detected")
        self.publisher.publish(msg)

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