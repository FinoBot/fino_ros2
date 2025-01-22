import os
import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
from vosk import Model, KaldiRecognizer
from std_msgs.msg import String
import json
import threading

class AudioRecognition(Node):
    def __init__(self):
        super().__init__('audio_recognition')
        self.get_logger().info(f'Current working directory: {os.getcwd()}')
        model_path = '/root/ros2_ws/src/fino_ros2/model/vosk-model-small-fr-0.22'
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 48000)
    
        self.audio_commands = self.create_publisher(String, '/audio_commands', 10)
    
        self.activation_threshold = 1000  # Adjust the threshold as needed
        self.max_queue_size = 10  # Set a maximum size for the audio queue

        self.get_logger().info('Audio Recognition Node has been started.')
        self.audio_queue = []

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    # def audio_callback(self, indata, frames, time, status):
    #     if status:
    #         self.get_logger().error(str(status))
        
    #     audio_data = np.frombuffer(indata, dtype=np.int16)
    #     self.audio_queue.append(audio_data)

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().error(str(status))
        
        audio_data = np.frombuffer(indata, dtype=np.int16)
        
        # Apply gain to amplify the audio
        gain = 3.0  # Adjust the gain factor as needed
        amplified_audio = np.clip(audio_data * gain, -32768, 32767).astype(np.int16)
        
        self.audio_queue.append(amplified_audio)
        if len(self.audio_queue) < self.max_queue_size:
            self.audio_queue.append(amplified_audio)
        else:
            self.get_logger().warning('Audio queue is full, dropping audio data')


    def process_audio(self):
        while rclpy.ok():
            if self.audio_queue:
                audio_data = self.audio_queue.pop(0)
                if self.recognizer.AcceptWaveform(audio_data.tobytes()):
                    result = json.loads(self.recognizer.Result())
                    self.get_logger().info(result["text"])
                    if result["text"] != "":
                        msg = String()
                        msg.data = result["text"]
                        self.audio_commands.publish(msg)

    def start_listening(self):
        threading.Thread(target=self.process_audio, daemon=True).start()
        with sd.RawInputStream(samplerate=48000, blocksize=2000, channels=1, dtype='int16', callback=self.audio_callback):
            self.get_logger().info('Listening...')
            rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecognition()
    node.start_listening()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()