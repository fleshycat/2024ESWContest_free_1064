from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import random
from threading import Thread
import time

app = Flask(__name__)
socketio = SocketIO(app)

# 초기 로봇 위치
robot_latitude = 37.7749
robot_longitude = -122.4194

# 로봇의 GPS 좌표를 업데이트하는 함수 (백그라운드 스레드에서 실행)
def update_robot_location():
    global robot_latitude, robot_longitude
    while True:
        # 일정 범위 내에서 랜덤하게 좌표 변경
        robot_latitude += random.uniform(-0.01, 0.01)
        robot_longitude += random.uniform(-0.01, 0.01)
        time.sleep(1)  # 1초마다 위치 업데이트

# 클라이언트에게 GPS 데이터를 전송하는 함수
def send_gps_data():
    while True:
        socketio.emit('gps_data', {'latitude': robot_latitude, 'longitude': robot_longitude})
        print(robot_latitude, robot_longitude)
        time.sleep(1)  # 1초마다 GPS 데이터 전송

# 루트 경로에 접속할 때 지도를 표시하는 HTML 파일을 렌더링
@app.route('/')
def index():
    return render_template('index.html')

# 클라이언트가 연결되었을 때 실행되는 이벤트 핸들러
@socketio.on('connect')
def handle_connect():
    print('클라이언트가 연결되었습니다.')

# 클라이언트가 연결 해제될 때 실행되는 이벤트 핸들러
@socketio.on('disconnect')
def handle_disconnect():
    print('클라이언트가 연결 해제되었습니다.')

if __name__ == '__main__':
    # 백그라운드 스레드에서 로봇의 위치 업데이트 및 GPS 데이터 전송 함수 실행
    Thread(target=update_robot_location).start()
    Thread(target=send_gps_data).start()
    socketio.run(app)
