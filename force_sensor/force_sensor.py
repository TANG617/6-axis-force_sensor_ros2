import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import struct

class ForceSensorNode(Node):

    def __init__(self):
        super().__init__('force_sensor_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'force_sensor', 10) # topic 为'force_sensor'
        timer_period = 1.0  # 1秒发布1次消息
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def tcp_client_send_data(self,server_ip, server_port, data):
       
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


        server_address = (server_ip, server_port)
        self.get_logger().info(f"正在连接到 {server_address}")
        sock.connect(server_address)

        try:
            self.get_logger().info(f"正在发送数据: {data}")
            sock.sendall(data)

            response = sock.recv(1024)
            data_fields = struct.unpack('>9x x B B 6f B B', response)
            self.get_logger().info(f"收到响应: {response}")

            data_format = {
                "Fx": data_fields[2],
                "Fy": data_fields[3],
                "Fz": data_fields[4],
                "Mx": data_fields[5],
                "My": data_fields[6],
                "Mz": data_fields[7],
            }

            for key, value in data_format.items():
                self.get_logger().info(f"{key}: {value:.7f}")
            self.get_logger().info("\n")
            return data_format

        finally:
            self.get_logger().info(f'关闭tcp连接')
            sock.close()

    def timer_callback(self):
        server_ip = '192.168.1.238'
        server_port = 502
        data = bytes([0x00, 0x01, 0x00, 0x02, 0x00, 0x0A, 0xf1, 0x09, 0x01, 0x04, 0x00, 0x54, 0x00, 0x0c, 0xb1, 0xdf])

        force_data = self.tcp_client_send_data(server_ip, server_port, data)
    
        msg = Float32MultiArray()
        msg.data = [force_data['Fx'], force_data['Fy'], force_data['Fz'], force_data['Mx'],force_data['My'],force_data['Mz']]
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布消息: {msg.data}')

def main(args=None):

    rclpy.init(args=args)
    force_sensor_node = ForceSensorNode()
    rclpy.spin(force_sensor_node)
    force_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
