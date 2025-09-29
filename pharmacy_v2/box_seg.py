# yolo_test_jsg.py — /box_start 수신 후 YOLO 감지 → XY 이동
import os
os.environ["CUDA_VISIBLE_DEVICES"] = ""  # 필요 시 제거

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation
import numpy as np
import threading
from queue import Queue, Empty

import DR_init
from .onrobot import RG
#------path------
from pathlib import Path


config_dir = Path(__file__).parent / "config"
waypoint_file = config_dir / "waypoint.json"
tgripper_file = config_dir / "T_gripper2camera.npy"
yolo_box_model_file = config_dir / "my_best_box_4.pt"
# tgripper_file = (Path.home()/"ros2_ws"/"src"/"DoosanBootcamp3rd"/"dsr_rokey"/"rokey"/"rokey"/"basic"/"config"/"T_gripper2camera.npy")
# yolo_box_model_file = (Path.home()/"ros2_ws"/"src"/"DoosanBootcamp3rd"/"dsr_rokey"/"rokey"/"rokey"/"basic"/"config"/"my_best_box_4.pt")
#------path------


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

class TestNode(Node):
    def __init__(self, wp, posx, posj, movej, movel, get_current_posx, wait):
        super().__init__("test_node")
        self.wp = wp
        self.posx = posx
        self.posj = posj
        self.movej = movej
        self.movel = movel
        self.get_current_posx = get_current_posx
        self.wait = wait

        # 실행 플래그
        self.box_requested = False
        self.target_name = None

        # --- ROS I/O ---
        self.bridge = CvBridge()
        self.pub_vis = self.create_publisher(Image, "/yolo/vis_image", 10)
        self.box_done_pub = self.create_publisher(String, '/box_done', 10)

        # /box_start 구독
        self.box_start_sub = self.create_subscription(String, '/box_start', self.box_start_cb, 10)

        self.sub_color = self.create_subscription(Image, "/camera/camera/color/image_raw", self.on_color, 10)
        self.sub_depth = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.on_depth, 10)
        self.sub_info  = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.on_info, 10)

        # --- Latest frames / intrinsics ---
        self.depth = None
        self.K = None  # {"fx","fy","ppx","ppy"}

        # --- Robot / Gripper ---
        self.gripper2cam = np.load(tgripper_file)
        self.JReady = self.posj([0, 0, 90, 0, 90, -90])
        self.see_box = self.wp.get_pos("see_box")
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # --- YOLO ---
        self.model = YOLO(yolo_box_model_file)
        names = self.model.names
        self.names = names if isinstance(names, dict) else {i: n for i, n in enumerate(names)}
        self.conf_thres = 0.25
        self.draw_conf_thres = 0.7

        # --- ROI/State ---
        self.roi_size = 300
        self.candidates = {}
        self.lock = threading.Lock()

        # --- Publisher thread ---
        self.pub_queue: Queue[tuple] = Queue(maxsize=5)
        self.pub_thread = threading.Thread(target=self._pub_worker, daemon=True)
        self.pub_thread.start()

        # --- Motion thread 상태 ---
        self.moving = False

    # ===================== topic pub sub ====================
    def box_start_cb(self, msg):
        self.target_name = msg.data.strip()
        self.box_requested = True
        self.get_logger().info(f"/box_start 수신 - 타겟: {self.target_name}")
        self.movel(self.see_box, vel=VELOCITY, acc=ACC)

    def send_done(self):
        done_msg = String()
        done_msg.data = "done"
        self.box_done_pub.publish(done_msg)
        self.get_logger().info("/box_done 발행: done")

    # ===================== Subscriptions =====================
    def on_info(self, msg: CameraInfo):
        self.K = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def on_color(self, msg: Image):
        if self.depth is None or self.K is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if img is None:
            return

        import cv2
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        x1, y1 = max(0, cx - self.roi_size // 2), max(0, cy - self.roi_size // 2)
        x2, y2 = min(w - 1, cx + self.roi_size // 2), min(h - 1, cy + self.roi_size // 2)

        res = self.model.predict(img, verbose=False, device="cpu", conf=self.conf_thres)[0]
        boxes = getattr(res, "boxes", None)

        cand = {}
        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                bx1, by1, bx2, by2 = map(int, boxes.xyxy[i].tolist())
                conf = float(boxes.conf[i].item()) if boxes.conf is not None else 0.0
                cls_id = int(boxes.cls[i].item()) if boxes.cls is not None else -1
                cls_name = self.names.get(cls_id, f"id_{cls_id}")
                u = (bx1 + bx2) // 2; v = (by1 + by2) // 2

                if x1 <= u <= x2 and y1 <= v <= y2 and conf >= self.draw_conf_thres:
                    prev = cand.get(cls_name)
                    if (prev is None) or (conf > prev[0]):
                        cand[cls_name] = (conf, (u, v), (bx1, by1, bx2, by2))

                if conf >= self.draw_conf_thres:
                    label = f"{cls_name} {conf*100:.1f}%"
                    cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                    cv2.putText(img, label, (bx1 + 2, by1 + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

        with self.lock:
            self.candidates = cand

        # /box_start 수신 후 타겟 발견 시 이동
        if self.box_requested and self.target_name and self.candidates:
            if self.target_name in self.candidates:
                self.get_logger().info(f"[DETECTED] 요청한 타겟 '{self.target_name}' 발견 → 이동 시도")
                self._try_move_if_match(self.target_name)
                self.box_requested = False
            else:
                self.get_logger().info(f"[WAIT] '{self.target_name}' 아직 미검출. 계속 탐색 중...")

        # rqt 시각화 발행
        try:
            if self.pub_queue.full():
                _ = self.pub_queue.get_nowait()
            self.pub_queue.put_nowait((img, msg.header))
        except Exception:
            pass

    # ===================== Publisher thread =====================
    def _pub_worker(self):
        while rclpy.ok():
            try:
                img, header = self.pub_queue.get(timeout=0.1)
            except Empty:
                continue
            msg_out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            msg_out.header = header
            msg_out.header.frame_id = "yolo_vis"
            self.pub_vis.publish(msg_out)

    # ===================== Try motion =====================
    def _try_move_if_match(self, user_cls: str):
        if user_cls in self.candidates:
            conf, (u, v), (bx1, by1, bx2, by2) = self.candidates[user_cls]
            u_c = (bx1 + bx2) // 2
            v_c = (by1 + by2) // 2
            if not self.moving:
                self.moving = True
                threading.Thread(target=self._move_xy_thread, args=(u_c, v_c, user_cls), daemon=True).start()
                self.get_logger().info(f"[OK] '{user_cls}' 일치(conf={conf:.3f}) → XY 이동 시작.")
            else:
                self.get_logger().info("[SKIP] 이미 이동 중.")
        else:
            self.get_logger().info(f"[NG] '{user_cls}' 미일치. 이동 취소.")

    def _move_xy_thread(self, u, v, user_cls: str):
        try:   
            self.gripper.move_gripper(500)
            self.move_to_img_xy(u, v, use_gripper=False)
            self._post_move_to_waypoint(user_cls)

            
            self.send_done() # 250815 ✅ 웨이포인트 이동까지 끝났을 때 완료로 정의
        finally:
            self.moving = False

    def _post_move_to_waypoint(self, user_cls: str):
        try:
            cls2wp = {
                'allergy':     ('box_left', 250),
                'famotidine':  ('box_center', 300), 
                'somnifacient':('box_right', 320)
            }
            key = cls2wp.get(user_cls)
            if not key:
                self.get_logger().info(f"[INFO] '{user_cls}' 웨이포인트 없음. 생략.")
                return
            self.gripper.move_gripper(500)
            target = self.wp.get_pos(key[0])
            self.movel(target, vel=30, acc=30)
            # 집기 동작 예시
            pos_list = list(self.posx(self.get_current_posx()[0]))
            pos_list[2] -= 50
            self.movel(self.posx(pos_list), vel=VELOCITY, acc=ACC)

            self.gripper.move_gripper(key[1])
            pos_list = list(self.posx(self.get_current_posx()[0]))
            pos_list[2] += 70
            self.movel(self.posx(pos_list), vel=VELOCITY, acc=ACC)
            pos_list = list(self.posx(self.get_current_posx()[0]))
            pos_list[1] += 40
            self.movel(self.posx(pos_list), vel=VELOCITY, acc=ACC)
            
        except Exception as e:
            self.get_logger().error(f"[ERR] 웨이포인트 이동 실패: {e}")

    # ===================== Kinematics =====================
    def get_camera_pos(self, u, v, z, K):
        x = (u - K["ppx"]) * z / K["fx"]
        y = (v - K["ppy"]) * z / K["fy"]
        return (x, y, z)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        Rm = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = [x, y, z]
        return T

    def cam_to_base(self, Xc):
        base2gripper = self.get_robot_pose_matrix(*self.get_current_posx()[0])
        base2cam = base2gripper @ self.gripper2cam
        Xc_h = np.append(np.array(Xc), 1.0)
        Xb = base2cam @ Xc_h
        return Xb[:3]

    def move_xy_to_base(self, x_b, y_b):
        cur = self.get_current_posx()[0]
        target = self.posx([x_b, y_b, cur[2], cur[3], cur[4], cur[5]])
        self.movel(target, 30, 30)

    def move_to_img_xy(self, u, v, use_gripper=False):
        if self.depth is None or self.K is None:
            return
        h, w = self.depth.shape
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))
        z = float(self.depth[v, u])
        Xc = self.get_camera_pos(u, v, z, self.K)
        Xb = self.cam_to_base(Xc)
        self.move_xy_to_base(Xb[0], Xb[1])
        if use_gripper:
            self.send_done()

def main(args=None):
    rclpy.init()
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_current_posx, movej, movel, wait
    from DR_common2 import posx, posj
    from .nav_waypoint import WaypointManager
    wp = WaypointManager(waypoint_file)

    app = TestNode(wp, posx, posj, movej, movel, get_current_posx, wait)
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(app)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
