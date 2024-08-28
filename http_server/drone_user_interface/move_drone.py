import math
import socket
import time

# 중심 좌표
center_lat = 36.625064
center_lon = 127.458123

# 원의 반지름 (미터)
radius = 500

# 1분에 이동하는 거리 (미터)
distance_per_minute = 1000

# 총 이동 시간 (분)
total_time_minutes = 60000000  # 예시로 60분 동안의 위치를 계산

# 지구의 반지름 (미터)
earth_radius = 6371000

# 서버 설정
UDP_IP = "127.0.0.1"  # 서버 IP 주소
UDP_PORT = 5005

# UDP 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 각도 변화량 계산 (라디안)
angle_change_per_minute = distance_per_minute / radius

for minute in range(total_time_minutes):
    # 현재 각도 계산
    angle = angle_change_per_minute * 0.1*minute

    # 현재 위치 계산 (반시계 방향)
    delta_lat = (radius * math.cos(angle)) / earth_radius * (180 / math.pi)
    delta_lon = (radius * math.sin(angle)) / (earth_radius * math.cos(center_lat * math.pi / 180)) * (180 / math.pi)

    # 새로운 위치 계산
    new_lat = center_lat + delta_lat
    new_lon = center_lon + delta_lon

    # 위치 데이터 문자열로 변환
    position_str = f"{new_lat},{new_lon}"
    print(f"전송할 위치: {position_str}")

    # 서버로 데이터 전송
    sock.sendto(position_str.encode(), (UDP_IP, UDP_PORT))

    # 1분 대기 (테스트 목적으로 1초로 줄임)
    time.sleep(0.1)

# 소켓 닫기
sock.close()
