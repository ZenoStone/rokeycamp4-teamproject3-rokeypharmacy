# go_main.py — 멀티스레드 + reentrant + 이벤트 기반 완료대기

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from pathlib import Path
import time
import threading

# ----------------- 환경 설정 -----------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

config_dir = Path(__file__).parent / "config"

WAYPOINT_FILE = config_dir / "waypoint.json"

GRIPPER_NAME="rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

LIST_TOPIC = ['/seg_start', '/box_start']
# --------------------------------------------


class GoBoxNode(Node):
    """토픽을 받아 시퀀스를 실행. DSR 함수들은 생성 시 주입."""
    def __init__(self,
                 movej, movel, check_force_condition, DR_AXIS_Z,
                 posx, posj,
                 WaypointManager,
                 drug_info, execute_from_key, current_move, current_move_two, into_capsule, gripper):
        super().__init__('go_box_node', namespace=ROBOT_ID)

        # DSR/유틸 함수 보관
        self._movej = movej
        self._movel = movel
        self._check_force_condition = check_force_condition
        self._DR_AXIS_Z = DR_AXIS_Z
        self._posx = posx
        self._posj = posj
        self._WaypointManager = WaypointManager
        self._drug_info = drug_info
        self._execute_from_key = execute_from_key
        self._current_move = current_move
        self._current_move_two = current_move_two
        self._gripper = gripper
        self._into_capsule = into_capsule

        # 상태
        self.running = False
        self.MD_NAME = ""

        # 완료 이벤트 (토픽 수신 → set)
        self.seg_done_event = threading.Event()
        self.box_done_event = threading.Event()

        # reentrant 콜백 그룹 (동시 콜백 허용)
        self.cb_group = ReentrantCallbackGroup()

        # pub/sub
        self.seg_pub = self.create_publisher(String, LIST_TOPIC[0], 10)
        self.box_pub = self.create_publisher(String, LIST_TOPIC[1], 10)

        self.create_subscription(String, '/seg_done', self.seg_done_cb, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(String, '/box_done', self.box_done_cb, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(String, '/go_main', self.md_name_cb, 10,
                                 callback_group=self.cb_group)

    # ---------- 콜백 ----------
    def seg_done_cb(self, msg: String):
        if msg.data.strip().lower() == "done":
            self.get_logger().info("pill_seg 완료 수신")
            self.seg_done_event.set()

    def box_done_cb(self, msg: String):
        if msg.data.strip().lower() == "done":
            self.get_logger().info("box_seg 완료 수신")
            self.box_done_event.set()

    # ---------- 퍼블리시 + 이벤트 대기 ----------
    def run_pill(self, info: str, timeout: float = 30.0):
        self.seg_done_event.clear()
        self.seg_pub.publish(String(data=info))
        self.get_logger().info(f"pill_seg 시작: {info}")
        ok = self.seg_done_event.wait(timeout=timeout)
        if not ok:
            raise TimeoutError("pill_seg 완료 신호(/seg_done) 타임아웃")

    def run_box(self, info: str, timeout: float = 60.0):
        self.box_done_event.clear()
        self.box_pub.publish(String(data=info))
        self.get_logger().info(f"box_seg 시작: {info}")
        ok = self.box_done_event.wait(timeout=timeout)
        if not ok:
            raise TimeoutError("box_seg 완료 신호(/box_done) 타임아웃")

    # ---------- /go_main 수신 → 별도 스레드 실행 ----------
    def md_name_cb(self, msg: String):
        name = msg.data.strip()
        self.get_logger().info(f"/go_main 수신: '{name}'")
        if not name:
            self.get_logger().warn("빈 약 이름. 생략.")
            return
        if self.running:
            self.get_logger().warn("이미 동작 중. 새로운 요청 무시.")
            return

        self.running = True
        threading.Thread(target=self._execute_flow_thread, args=(name,), daemon=True).start()

    def _execute_flow_thread(self, name: str):
        try:
            self.execute_flow(name)
        except Exception as e:
            self.get_logger().error(f"동작 예외: {e}")
        finally:
            self.running = False

    # ---------- 실제 시퀀스 ----------
    def execute_flow(self, name: str):
        # 유틸 준비
        wp = self._WaypointManager(WAYPOINT_FILE)
        home = wp.get_pos("home")
        client = wp.get_pos("client")

        # 홈 이동
        self.get_logger().info("홈 포즈 이동")
        self._gripper.move_gripper(500)
        self._gripper.move_gripper(300)
        self._movej(home, vel=VELOCITY, acc=ACC)

        # 약 정보 조회
        self.get_logger().info(f"약 정보 조회: {name}")
        info = self._drug_info(name)

        # 분기점(추후 약 정보 여러개 넣을 경우 queue 해서 처리해도 됨)
        if not info:
            self.get_logger().error("해당 약 이름 waypoint 없음")
            return

        if info['group'] == 'list_drug':
            # 알약 시나리오 (알약은 무조건 qr?)
            self._execute_from_key(info['pos'], step="pick")
            self.run_pill(name)
            self._execute_from_key(info['pos'], step="place")
            self._movej(home, vel=VELOCITY, acc=ACC)
            self._into_capsule()

        elif info['group'] == 'list_box':
            # 박스 시나리오: box_seg에 작업 위임 후 완료 토픽을 이벤트로 기다림
            self.get_logger().info("박스 동작 테스트")
            self.run_box(name)

        # 고객 봉투 상부로 이동 후 내려놓기
        self.get_logger().info("봉투로")
        self._movel(client, vel=VELOCITY, acc=ACC)
        self._current_move(2, -100)
        self._gripper.move_gripper(500)
        time.sleep(1)
        self._gripper.move_gripper(300)
        self.get_logger().info("최종 투입 후 홈 복귀")
        self._movej(home, vel=VELOCITY, acc=ACC)


def main(args=None):
    rclpy.init(args=args)

    # DSR core node 먼저 생성 → DR_init 바인딩 → DSR 임포트(한 번만)
    import DR_init as DRI
    dsr_core = rclpy.create_node("dsr_core", namespace=ROBOT_ID)
    DRI.__dsr__node  = dsr_core
    DRI.__dsr__id    = ROBOT_ID
    DRI.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import movej, movel, check_force_condition, DR_AXIS_Z
    from DR_common2 import posx, posj

    from .nav_waypoint import WaypointManager
    from .auto_move import drug_info, execute_from_key, current_move, current_move_two, into_capsule
    from .onrobot import RG
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    app = GoBoxNode(
        movej, movel, check_force_condition, DR_AXIS_Z,
        posx, posj,
        WaypointManager,
        drug_info, execute_from_key, current_move, current_move_two, into_capsule, gripper
    )

    # 멀티스레드 실행기: 콜백 동시성 확보
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(app)
    executor.add_node(dsr_core)

    try:
        executor.spin()
    finally:
        app.destroy_node()
        dsr_core.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
