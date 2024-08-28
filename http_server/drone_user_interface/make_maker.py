import math
import socket
import time

# 중심 좌표
center_lat = 36.625064
center_lon = 127.458123

# 서버 설정
UDP_IP = "127.0.0.1"  # 서버 IP 주소
UDP_PORT = 5010

# UDP 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    
    delta_lat, delta_lon=input().split(", ")
    delta_lat=float(delta_lat)
    delta_lon=float(delta_lon)
    
    lat=center_lat - 0.01*delta_lat
    lon=center_lon - 0.01*delta_lon
    # 위치 데이터 문자열로 변환
    position_str = f"{lat},{lon}"
    print(f"전송할 위치: {position_str}")

    # 서버로 데이터 전송
    sock.sendto(position_str.encode(), (UDP_IP, UDP_PORT))

# 소켓 닫기
sock.close()
