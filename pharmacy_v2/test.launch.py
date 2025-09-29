from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rokey', executable='ready', name='go_main_node', output='screen', emulate_tty=True),
        Node(package='rokey', executable='box', name='box_seg_node', output='log', emulate_tty=True),
        Node(package='rokey', executable='pill', name='pill_seg_node', output='log', emulate_tty=True),
        
    ])

'''
일단은 basic 이후에 launch 디렉토리 생성
디렉토리에 test.launch.py(파일 이름에 .launch.py가 있어야만 함) 생성 후 상단 처럼 작성
다만 executable 은 entry_point에 있는 이름과 동일하게 작성
output='screen' >>> 터미널에 output
output='log'    >>> 로그에 output
name과 namespace 같은 경우 생략해도 문제 없음

setup.py 의 최상단에
from setuptools import find_packages, setup
import os
from glob import glob
추가
------------------------------
data_files 에
(os.path.join('share', package_name, 'launch'), glob('rokey/basic/launch/*.launch.py')),
추가


이후 
colcon build --packages-select rokey
source ~/ros2_ws/install/setup.bash
실행

터미널에 ros2 launch rokey t 
이후 탭으로 나오는지 확인(상단에 파일 이름을 임의로 작성했다면 본인이 설정한 대로 나옴 걱정ㄴㄴ)
'''