# 같은 src 내에 pharmacy_interface 필요

# medicine_send4.py 와 medicine_server.py 노드를 실행해서 진행

> medicine_send4.py 는 서비스 클라이언트
> medicine_server.py 는 서비스 서버

# 실행 방법

터미널 2개 키기, 각각 패키지 파일로 cd 한 다음 colcon build, sorce install/setup.bash 
만약 colcon build에서 오류 발생 시, colcon build --packages-select pharmacy_interface 부터 진행 한 후 전체 colcon


> terminal 1 export [제공받은 오픈 API키 입력]
> terminal 1 ros2 run meditts medicine_send4 


> terminal 2 ros2 run meditts medicine_server 



# 구동을 위해서 필요한 파일
```python
MicController.py
STT.py
STT_QR_connect.py
hello_rokey_8332_32.tflite
```

# STT_QR_connect.py 파일 내 hello_rokey_8332_32.tflite 경로 설정 필요!
MODEL_PATH = "/home/rokey/hubtest_ws/src/meditts/meditts/hello_rokey_8332_32.tflite" ### hello_rokey_8332_32.tflite 경로 설정 필요

# API 키
같은 경로에 아래의 .env로 API키 필요
[제공받은 오픈 API키 입력]
