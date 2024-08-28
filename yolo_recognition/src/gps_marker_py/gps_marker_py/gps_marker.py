# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO # YOLO library
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import socket
from math import radians, cos, sin, sqrt, atan2

# Load the YOLOv8 model
model = YOLO('yolov8m.pt')

# 서버 설정
UDP_IP = "127.0.0.1"  # 서버 IP 주소
UDP_PORT = 5010

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
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
    self.last_marker_gps_position=[0,0]
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    image = current_frame
    
    # Object Detection
    results = model.predict(image, classes=[0, 2])

    for detection in results[0].boxes:
      if self.check_distance() >= 10:
        self.send_marker_LLH()
        self.last_marker_gps_position[0] = self.lat
        self.last_marker_gps_position[1] = self.lon
        
    img = results[0].plot()
    # Show Results
    cv2.imshow('Detected Frame', img)    
    cv2.waitKey(1)
    
  def gps_callback(self, msg):
      self.lat=msg.lat
      self.lon=msg.lon
      
  def send_marker_LLH(self):   
    lat = self.lat
    lon = self.lon
    # UDP 소켓 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    position_str = f"{lat},{lon}"
    print(f"전송할 마커 위치: {position_str}")

    # 서버로 데이터 전송
    sock.sendto(position_str.encode(), (UDP_IP, UDP_PORT))
    # 소켓 닫기
    sock.close()
    
  def check_distance(self):
    R = 6371000

    # 위도와 경도를 라디안 단위로 변환
    lat1_rad = radians(self.lat)
    lon1_rad = radians(self.lon)
    lat2_rad = radians(self.last_marker_gps_position[0])
    lon2_rad = radians(self.last_marker_gps_position[1])

    # 위도와 경도의 차이
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine 공식을 사용하여 거리 계산
    a = sin(dlat / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R * c
    return distance   
        
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  # Spin the node so the callback function is called.
  
  rclpy.spin(image_subscriber)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
