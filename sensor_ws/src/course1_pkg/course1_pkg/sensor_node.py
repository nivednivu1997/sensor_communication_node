import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import socket
import struct
import threading
from std_msgs.msg import Float32, Int32, Bool
from sensor_msgs.msg import Temperature, Imu
from std_srvs.srv import Empty

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameter('sensor_ip', '192.168.1.1')  # Default sensor IP
        self.declare_parameter('sensor_port', 2000)  # Default sensor port
        self.declare_parameter('interval', 1000)  # Default interval in milliseconds

        self.sensor_ip = self.get_parameter('sensor_ip').value
        self.sensor_port = self.get_parameter('sensor_port').value
        self.interval = self.get_parameter('interval').value

        # Publishers
        self.voltage_pub = self.create_publisher(Float32, 'supply_voltage', 10)
        self.temp_pub = self.create_publisher(Temperature, 'env_temp', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)

        # TCP connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.sensor_ip, self.sensor_port))

        # Automatically send the Start Command on launch
        self.send_start_command(self.interval)
        self.get_logger().info(f"Sensor started with interval {self.interval} ms.")

        # Services
        self.start_service = self.create_service(
            Int32, 'start_sensor', self.start_sensor_callback)
        self.stop_service = self.create_service(
            Empty, 'stop_sensor', self.stop_sensor_callback)

        # Thread for receiving data
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def start_sensor_callback(self, request, response):
        """
        Callback for the Start Service.
        Starts the sensor with the specified interval.
        """
        interval = request.data
        if interval <= 0:
            response.success = False
            self.get_logger().error("Interval must be a positive integer.")
            return response

        try:
            self.send_start_command(interval)
            response.success = True
            self.get_logger().info(f"Sensor started with interval {interval} ms.")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to start sensor: {e}")

        return response

    def stop_sensor_callback(self, request, response):
        """
        Callback for the Stop Service.
        Stops the sensor.
        """
        try:
            self.send_stop_command()
            response.success = True
            self.get_logger().info("Sensor stopped.")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to stop sensor: {e}")

        return response

    def send_start_command(self, interval):
        """
        Sends a Start Command to the sensor with the specified interval.
        """
        command = b'#' + bytes.fromhex('03') + struct.pack('<H', interval) + b'\r\n'
        self.sock.sendall(command)

    def send_stop_command(self):
        """
        Sends a Stop Command to the sensor.
        """
        command = b'#' + bytes.fromhex('09') + b'\r\n'
        self.sock.sendall(command)

    def receive_data(self):
        """
        Thread function to receive data from the sensor.
        """
        while True:
            data = self.sock.recv(1024)
            if data.startswith(b'$'):
                self.decode_status_message(data)

    def decode_status_message(self, data):
        """
        Decodes a status message from the sensor and publishes the data.
        """
        if len(data) < 14:  # Minimum length for status message
            return
        command_id = data[1:3]
        if command_id != bytes.fromhex('11'):
            return
        payload = data[3:-2]  # Remove start, command ID, and end
        supply_voltage = struct.unpack('<H', payload[0:2])[0]
        env_temp = struct.unpack('<h', payload[2:4])[0]
        yaw = struct.unpack('<h', payload[4:6])[0]
        pitch = struct.unpack('<h', payload[6:8])[0]
        roll = struct.unpack('<h', payload[8:10])[0]

        # Publish data
        self.voltage_pub.publish(Float32(data=supply_voltage / 1000.0))  # Convert to volts
        temp_msg = Temperature()
        temp_msg.temperature = env_temp / 10.0  # Convert to Celsius
        self.temp_pub.publish(temp_msg)
        imu_msg = Imu()
        imu_msg.orientation = self.euler_to_quaternion(yaw / 10.0, pitch / 10.0, roll / 10.0)
        self.imu_pub.publish(imu_msg)

    def euler_to_quaternion(self, yaw, pitch, roll):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x,y,z,w]

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
