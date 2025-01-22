import os
import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import speech_recognition as sr
from std_msgs.msg import String
import threading

wordlist = ["suis-moi", "stop", "debout", "coucou", "salut", "hey"]

class AudioRecognition(Node):
    def __init__(self):
        super().__init__('audio_recognition')
        self.get_logger().info(f'Current working directory: {os.getcwd()}')
        
        self.recognizer = sr.Recognizer()
        self.audio_commands = self.create_publisher(String, '/audio_commands', 10)

        self.get_logger().info('Audio Recognition Node has been started.')
        self.audio_queue = []

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().error(str(status))
        
        audio_data = np.frombuffer(indata, dtype=np.int16)
        self.audio_queue.append(audio_data)

    def process_audio(self):
        while rclpy.ok():
            if self.audio_queue:
                audio_data = self.audio_queue.pop(0)
                audio_data = audio_data.tobytes()
                audio = sr.AudioData(audio_data, 48000, 2)
                try:
                    result = self.recognizer.recognize_sphinx(audio, keyword_entries=[(word, 1.0) for word in wordlist])
                    self.get_logger().info(result)
                    if result != "":
                        msg = String()
                        msg.data = result
                        self.audio_commands.publish(msg)
                except sr.UnknownValueError:
                    self.get_logger().info("Sphinx could not understand audio")
                except sr.RequestError as e:
                    self.get_logger().error(f"Sphinx error; {e}")

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