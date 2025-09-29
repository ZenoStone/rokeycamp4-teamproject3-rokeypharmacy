# rokeycamp4-teamproject3-rokeypharmacy
로키 부트캠프 4기 협동2 프로젝트 결과물 입니다

## 구성 설명
사용하는 meditts, pharmacy_v2, launch 패키지 파일 사용
자체 제작 인터페이스 인 pharmacy_interface가 존재. 이것은 /ros2_ws/src 안에 배치

pharmacy_interface를 제외한 파일은 DR__init__ 때문에 ros2_ws 안의 /ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/rokey/rokey 경로에 배치

meditts는 stt 동작
> STT_QR_connect.py, medicine_send.py, medicine_server.py 그외는 모두 실행을 위해서 필요한 파일

pharmacy_v2 로봇 노드 동작
> go_main.py, pill_seg.py, box_seg.py 그외는 모두 실행을 위해서 필요한 파일

launch는 로봇 노드를 launch로 묶어서 한 번에 실행할 수 있는 파일
> test.launch.py



## 설정 방법
setup.py 안에 launch 파일을 위한 경로 설정

```python
setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
 
 	# --- 중략 ---
        ### launch 를 위한 경로
        (os.path.join('share', package_name, 'launch'), glob('rokey/launch/*.launch.py')),
	# --- 중략 ---
	],
	
	# --- 중략 ---
	entry_points={
        "console_scripts": [


	    # --- 중략 ---

            
            "medicine_send = rokey.meditts.medicine_send:main",
            "medicine_server = rokey.meditts.medicine_server:main",
            


            ### launch
            "pill_seg = rokey.pharmacy_v2.pill_seg:main",
            "box_seg = rokey.pharmacy_v2.box_seg:main",
            
            "go_main = rokey.pharmacy_v2.go_main:main",



        ],
    },
)	
	
```


## 실행 방법

> colcon build
> source install/setup.bash

터미널에 test.launch.py 와 medicine_send, medicine_server를 실행해서 시연 가능
! 자체 제작 인터페이스를 사용하기 때문에 colcon build 시 pharmacy_interface를 먼저 --packages-select로 진행한 다음 나머지 colcon build 하는 걸 추천함
- 터미널에 로봇의 브링업, realsense를 실행한 다음 진행해야 함

## License
This project is licensed under the MIT License. See the LICENSE file for details.