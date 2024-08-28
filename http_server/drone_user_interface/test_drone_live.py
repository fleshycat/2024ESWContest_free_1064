from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import socket
from threading import Thread
import os

app = Flask(__name__)
socketio = SocketIO(app)

# drone_pos_UDP 소켓 설정
UDP_IP = "127.0.0.1"
drone_pos_UDP_PORT = 5005
drone_pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_pos_sock.bind((UDP_IP, drone_pos_UDP_PORT))

# maker_pos_udp 소켓 설정
marker_pos_UDP_PORT = 5010
marker_pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
marker_pos_sock.bind((UDP_IP, marker_pos_UDP_PORT))

app = Flask(__name__)
socketio = SocketIO(app)

points = []

class Pointer:
    def __init__(self, lat, lng, index):
        self.lat = lat
        self.lng = lng
        self.index = index
        self.image_path = None

# 이미지 파일을 저장할 디렉토리 경로
UPLOAD_FOLDER = 'images'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

@app.route('/')
def index():
    return render_template('test_live_pos.html')

#중간에 접속한 인원에게도 기존 포인트 표시
@socketio.on('connect')
def handle_connect():
    emit('all_points', points)
    
#새로운 포인트 추가
@socketio.on('add_point')
def handle_add_point(data):
    lat = data['lat']
    lng = data['lng']
    index=next_index(points)
    point = Pointer(lat, lng, index)
    points.append(point)
    emit('new_point', {'lat': point.lat, 'lng': point.lng, 'image_path': point.image_path, 'index': index}, broadcast=True)
    print(points.index, sep='\n')
    print("_______________")
        
def next_index(points):
    if not points:
        return 0
    max_index = max(point.index for point in points)
    if max_index >= 2**31 - 1:  # 오버플로우를 방지하기 위한 최대값 (32-bit int 최대값)
        reset_indices(points)
        return len(points)  # 재설정 후 새로운 인덱스는 포인트의 개수와 동일
    return max_index + 1
#인덱스 오버플로우 발생시 인덱스 재정렬
def reset_indices(points):
    # 포인트들의 인덱스를 0부터 순차적으로 재설정
    for i, point in enumerate(points):
        point.index = i
    # 클라이언트들에게 업데이트된 포인트 전송
    for point in points:
        emit('new_point', point, broadcast=True)

# 포인트 삭제
@socketio.on('delete_point')
def handle_delete_point(index):
    for point in points:
        if str(point.index) == index:
            points.remove(point)
            emit('delete_point', index, broadcast=True)
    # print(*points, sep='\n')
    # print("_______________")
    
@app.route('/upload_image', methods=['POST'])
def upload_image():
    if 'image' in request.files:
        image = request.files['image']
        if image.filename != '':
            image_path = os.path.join(app.config['UPLOAD_FOLDER'], image.filename)
            image.save(image_path)
            return 'Image uploaded successfully'
    return 'Image upload failed'

# 포인트 클릭 시 해당 이미지 띄우기
@socketio.on('existing_point_clicked')
def handle_existing_point_clicked(data):
    # 해당 인덱스에 해당하는 사진 파일 경로 찾기
    image_filename = f"images/{data['index']}.png"
    if os.path.exists(image_filename):
        with open(image_filename, "rb") as image_file:
            image_data = image_file.read()
        print(image_filename)
        # 클라이언트로 이미지 데이터 전송
        emit('receive_image', {'image_data': image_data, 'image_name': os.path.basename(image_filename)}, broadcast=True)
        
# 클라이언트에게 GPS 데이터를 전송하는 함수
def send_gps_data():
    while True:
        data, addr = drone_pos_sock.recvfrom(1024) # UDP로부터 데이터 수신
        latitude, longitude = data.decode().split(',') # 받은 데이터 파싱
        socketio.emit('drone_pos', {'latitude': float(latitude), 'longitude': float(longitude)})
        #print("http", latitude, longitude)
        
def send_marker_data():
    while True:
        data, addr = marker_pos_sock.recvfrom(1024)
        latitude, longitude = data.decode().split(',')
        index=next_index(points)
        point = Pointer(latitude, longitude, index)
        points.append(point)
        socketio.emit('new_point', {'lat': point.lat, 'lng': point.lng, 'image_path': point.image_path, 'index': index})
        print("marker", latitude, longitude)

if __name__ == '__main__':
    # 백그라운드 스레드에서 GPS 데이터 전송 함수 실행
    Thread(target=send_gps_data).start()
    Thread(target=send_marker_data).start()
    socketio.run(app, host='0.0.0.0', port=5000)
