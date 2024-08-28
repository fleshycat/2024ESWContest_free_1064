import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class GPSListener(Node):

    def __init__(self):
        super().__init__('gps_listener')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_callback,
            qos_profile)
        self.subscription  # avoid unused variable warning

    def gps_callback(self, msg):
        #self.get_logger().info('Received GPS position: lat=%f, lon=%f, alt=%f' %(msg.lat, msg.lon, msg.alt))
        gps_position_str=f"{msg.lat},{msg.lon}"
        print(f"전송할 위치 {gps_position_str}")
        sock.sendto(gps_position_str.encode(), (UDP_IP, UDP_PORT))

def main(args=None):
    rclpy.init(args=args)
    gps_listener = GPSListener()
    rclpy.spin(gps_listener)
    gps_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
