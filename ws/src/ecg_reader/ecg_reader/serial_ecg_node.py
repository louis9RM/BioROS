import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ECGSerialNode(Node):
    def __init__(self):
        super().__init__('ecg_serial_node')
        self.publisher_ = self.create_publisher(Float32, 'ecg_raw', 10)
        port = '/dev/ttyACM0'
        baud = 115200
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Leyendo datos de {port} a {baud} baudios')
        except Exception as e:
            self.get_logger().error(f'Error al abrir puerto serial: {e}')
            self.ser = None
        self.timer = self.create_timer(0.004, self.timer_callback)  # ~250 Hz

    def timer_callback(self):
        if not self.ser:
            return
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.isdigit():
                msg = Float32()
                msg.data = float(line)
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Error leyendo: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ECGSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
