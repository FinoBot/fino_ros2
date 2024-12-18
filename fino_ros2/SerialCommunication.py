import serial

class SerialCommunication:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None

    def open_connection(self):
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout
            )
            print(f"Connection opened on {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Failed to open connection: {e}")

    def close_connection(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Connection closed.")

    def write(self, message):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(message)

    def read_messages(self):
        if self.serial_connection and self.serial_connection.is_open:
            message = self.serial_connection.readline().decode().strip()
            print(f"Received: {message}")
            return message