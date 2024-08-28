import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import socket
import time
import threading

# 서버 설정
UDP_IP = "127.0.0.1"  # 서버 IP 주소
UDP_PORT = 5010

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
        self.lat=0
        self.lon=0

    def gps_callback(self, msg):
        self.lat=msg.lat
        self.lon=msg.lon

def main(args=None):
    rclpy.init(args=args)
    gps_listener = GPSListener()
    
    thread = threading.Thread(target=send_marker_LLH, args=(gps_listener,))
    thread.daemon = True  # 메인 스레드 종료 시 함께 종료
    thread.start()
    
    rclpy.spin(gps_listener)
    gps_listener.destroy_node()
    rclpy.shutdown()

def send_marker_LLH(gps_listener):
    while True:
        # 10초 대기
        time.sleep(10)
        
        lat = gps_listener.lat
        lon = gps_listener.lon
        # UDP 소켓 생성
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        position_str = f"{lat},{lon}"
        print(f"전송할 마커 위치: {position_str}")

        # 서버로 데이터 전송
        sock.sendto(position_str.encode(), (UDP_IP, UDP_PORT))
        
        # 소켓 닫기
        sock.close()

if __name__ == '__main__':
    main()