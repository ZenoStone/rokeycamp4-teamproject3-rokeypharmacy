import os
os.environ["CUDA_VISIBLE_DEVICES"] = ""  # GPU 비활성화 (필요 시 제거)

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .realsense import ImgNode
from scipy.spatial.transform import Rotation
from .onrobot import RG
from ultralytics import YOLO
import time
import numpy as np
import DR_init
from rclpy.executors import MultiThreadedExecutor
#------path------
from pathlib import Path
config_dir = Path(__file__).parent / "config"

tgripper_file = config_dir / "T_gripper2camera.npy"
yolo_pill_model_file = config_dir / "my_best_pill_2.pt"
# tgripper_file = (Path.home()/"ros2_ws"/"src"/"DoosanBootcamp3rd"/"dsr_rokey"/"rokey"/"rokey"/"basic"/"config"/"T_gripper2camera.npy")
# yolo_pill_model_file = (Path.home()/"ros2_ws"/"src"/"DoosanBootcamp3rd"/"dsr_rokey"/"rokey"/"rokey"/"basic"/"config"/"my_best_pill_2.pt")
#------path------

# 로봇 및 그리퍼 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"


class SegNode(Node):
    def __init__(self, posx, posj, movej, movel, get_current_posx, wait):
        super().__init__("rokey_seg_test_node", namespace=ROBOT_ID)
        self.posx = posx
        self.posj = posj
        self.movej = movej
        self.movel = movel
        self.get_current_posx = get_current_posx
        self.wait = wait

        # 재진입 방지
        self.is_busy = False

        # YOLO
        self.model = YOLO(yolo_pill_model_file)

        # 카메라 설정
        self.img_node = ImgNode()
        self.intrinsics = None
        while self.intrinsics is None:
            rclpy.spin_once(self.img_node, timeout_sec=0.1)
            self.intrinsics = self.img_node.get_camera_intrinsic()
            if self.intrinsics is None:
                self.get_logger().warn("카메라 intrinsic 대기 중...")
                time.sleep(0.1)

        # 변환 행렬
        self.gripper2cam = np.load(tgripper_file)

        # 준비 자세
        self.JReady = self.posj([0, 0, 90, 0, 90, -90])

        # 그리퍼
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # ROS2 Pub/Sub
        self.seg_start_sub = self.create_subscription(String, '/seg_start', self.seg_start_cb, 10)
        self.seg_done_pub = self.create_publisher(String, '/seg_done', 10)

        self.target_name = None
        self.seg_requested = False
        self.get_logger().info("SegNode 초기화 완료. /seg_start 수신 대기 중...")

        # 메인 루프: 콜백에서 run() 호출하지 않고 타이머에서만 실행
        self.timer = self.create_timer(0.05, self.tick)

    # ====== 콜백 ======
    def seg_start_cb(self, msg):
        if self.is_busy:
            self.get_logger().warn("이미 작업 처리 중입니다. 새 요청은 무시됩니다.")
            return
        self.target_name = msg.data.strip()
        self.seg_requested = True
        self.get_logger().info(f"/seg_start 수신 - 타겟: {self.target_name}")

    def send_done(self):
        done_msg = String()
        done_msg.data = "done"
        self.seg_done_pub.publish(done_msg)
        self.get_logger().info("/seg_done 발행: done")

    # ====== 유틸 ======
    def get_camera_pos(self, center_x, center_y, center_z, intrinsics):
        camera_x = (center_x - intrinsics["ppx"]) * center_z / intrinsics["fx"]
        camera_y = (center_y - intrinsics["ppy"]) * center_z / intrinsics["fy"]
        return camera_x, camera_y, center_z

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords):
        coord = np.append(np.array(camera_coords), 1)

        # 최대 20회까지 재시도하며 로봇 포즈를 읽음
        for attempt in range(20):
            posx_info = self.get_current_posx()
            if posx_info and isinstance(posx_info[0], (list, tuple)) and len(posx_info[0]) >= 6:
                break
            self.get_logger().warn(f"[{attempt+1}/20] 현재 포즈 정보가 비어 있습니다. 재시도 중...")
            time.sleep(0.1)
        else:
            self.get_logger().error("로봇 포즈 정보를 얻지 못했습니다. 좌표 변환 실패.")
            return None

        base2gripper = self.get_robot_pose_matrix(*posx_info[0])
        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def get_depth_value(self, center_x, center_y, depth_frame):
        height, width = depth_frame.shape
        if 0 <= center_x < width and 0 <= center_y < height:
            return depth_frame[int(center_y), int(center_x)]
        return None

    def pick_procedure(self, x, y, z, angle):
        self.get_logger().info(f"피킹 시작: 위치=({x:.2f}, {y:.2f}, {z:.2f}), 각도={angle:.2f}")

        current_pos = self.get_current_posx()[0]
        target_orientation = [current_pos[3], current_pos[4], angle]

        # 상단 이동 (현재 Z 유지)
        above_pos = self.posx([x, y, current_pos[2], target_orientation[0], target_orientation[1], target_orientation[2]])
        self.movel(above_pos, VELOCITY, ACC)

        self.gripper.move_gripper(300)
        self.wait(1)
        
        # 피킹 위치 (오프셋)
        ### zaide의 경우 알약 자체의 빛반사로 오프셋 값을 더 줘야 함
        if self.target_name == "zaide":
            pick_pos = self.posx([x, y, z + 13, target_orientation[0], target_orientation[1], target_orientation[2]])
        else:
            pick_pos = self.posx([x, y, z + 10, target_orientation[0], target_orientation[1], target_orientation[2]])

        self.movel(pick_pos, VELOCITY / 2, ACC / 2)
        self.wait(1)

        self.gripper.move_gripper(150)
        self.wait(1)

        # 살짝 리프트
        lift_pos = self.posx([x, y, current_pos[2], target_orientation[0], target_orientation[1], target_orientation[2]])
        self.movel(lift_pos, VELOCITY, ACC)
        self.wait(1)

        self.get_logger().info("피킹 동작 완료.")

    # ====== 메인 루프(타이머) ======
    def tick(self):
        # 카메라 프레임 갱신
        rclpy.spin_once(self.img_node, timeout_sec=0.0)

        if not self.seg_requested or self.is_busy:
            return

        self.is_busy = True
        try:
            color_frame = self.img_node.get_color_frame()
            depth_frame = self.img_node.get_depth_frame()
            if color_frame is None or depth_frame is None:
                self.get_logger().warn("유효한 프레임이 없습니다. 다음 tick에서 재시도.")
                return

            results = self.model(color_frame, verbose=False)

            target_info = None
            if results and results[0].masks is not None:
                masks = results[0].masks.xy
                confs = results[0].boxes.conf
                classes = results[0].boxes.cls
                names_map = results[0].names

                detected_names = [names_map[int(c)] for c in classes]
                self.get_logger().info(f"Detected classes: {detected_names}")

                confident_indices = [i for i, conf in enumerate(confs) if conf >= 0.7]
                target_indices = [
                    i for i in confident_indices
                    if names_map[int(classes[i])].lower() == self.target_name.lower()
                ]

                if target_indices:
                    largest_contour = max([masks[i] for i in target_indices], key=cv2.contourArea)
                    (cx, cy), (w, h), angle = cv2.minAreaRect(largest_contour)
                    final_angle = angle if h < w else angle - 90
                    target_info = {"center_x": cx, "center_y": cy, "angle": final_angle}

            if target_info is not None:
                z = self.get_depth_value(target_info["center_x"], target_info["center_y"], depth_frame)
                if z is None or z == 0:
                    self.get_logger().warn("깊이 값 없음. 다시 탐색합니다.")
                    return

                camera_pos = self.get_camera_pos(target_info["center_x"], target_info["center_y"], z, self.intrinsics)
                robot_coord = self.transform_to_base(camera_pos)
                if robot_coord is None:
                    self.get_logger().warn("좌표 변환 실패. 다음 tick에서 재시도합니다.")
                    return

                self.pick_procedure(robot_coord[0], robot_coord[1], robot_coord[2], target_info["angle"])
                self.send_done()
                self.get_logger().info("피킹 요청 완료, 대기 상태로 복귀.")
                # 상태 초기화
                self.seg_requested = False
                self.target_name = None
            else:
                self.get_logger().info("타겟 객체를 찾지 못했습니다. 계속 탐색 중...")
        finally:
            self.is_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dummy_init", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_current_posx, movej, movel, wait
    from DR_common2 import posx, posj

    seg_node = SegNode(posx, posj, movej, movel, get_current_posx, wait)

    try:
        executor = MultiThreadedExecutor(num_threads=2)   # 2~3 권장
        executor.add_node(seg_node)                       # 세그 노드
        executor.add_node(seg_node.img_node)              # <<< 카메라 노드 함께 등록
        executor.add_node(node)                           # DSR dummy
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        seg_node.destroy_node()
        seg_node.img_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
