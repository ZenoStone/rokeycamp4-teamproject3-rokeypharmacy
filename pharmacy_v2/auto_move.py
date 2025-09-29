'''
v6 변경점

execute_from_key에 dz_close 를 추가해서
4_1, 4_2 만 y 좌표를 움직여서 닫히게 수정

current_move_two 추가로 2개의 좌표를 동시에 움직일 수 있음
--------------------------
pharmacy_v1 변경점

모든 동작 뒤에 
mwait()으로 강제 동기 적용
'''


from DR_common2 import posx, posj
from .onrobot import RG
import time
from DSR_ROBOT2 import movel, get_current_posx, check_force_condition, DR_AXIS_Z, movej, mwait
from .nav_waypoint import WaypointManager
from .basic_def import force_start, force_end
import json

#------path------
from pathlib import Path
config_dir = Path(__file__).parent / "config"

waypoint_file = config_dir / "waypoint.json"
wp = WaypointManager(waypoint_file)
#------path------

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

VELOCITY, ACC = 60, 60


def _resolve_down_name(base: str) -> str:
    base = base.strip()
    if base.startswith("pos"):
        name = base if base.endswith("_down") else f"{base}_down"
    else:
        name = f"pos{base}_down"
    return name


def make_variants(base: str, dz: float, dx: float, dy: float, dx_close: float, dz_close: float):
    down_name = _resolve_down_name(base)
    wp_entry = wp.get(down_name)
    if wp_entry is None:
        raise ValueError(f"'{down_name}' 을(를) JSON에서 찾을 수 없습니다.")

    x0, y0, z0, rx, ry, rz = wp_entry["value"]

    poses = {
        "down":        posx([x0,        y0,        z0,        rx, ry, rz]),
        "up":          posx([x0,        y0,        z0 + dz,   rx, ry, rz]),
        "down_x":      posx([x0 + dx,   y0,        z0,        rx, ry, rz]),
        "up_x":        posx([x0 + dx,   y0,        z0 + dz,   rx, ry, rz]),
        "see_pill":    posx([x0 + dx,   y0 + dy,   z0 + dz + 30,   rx, ry, rz]),
        "close_pose":  posx([x0 + dx,   y0 + dy,   z0 + dz_close,        rx, ry, rz]),
        "close":       posx([x0 + dx + dx_close, y0 + dy, z0 + dz_close, rx, ry, rz]),
    }
    return poses


# === 1단계: 픽업 ===
def run_sequence_pick(base, dz, dx, dy, dx_close, dz_close):
    p = make_variants(base, dz, dx, dy, dx_close, dz_close)
    movel(p["up"],         vel=VELOCITY, acc=ACC)
    mwait()
    gripper.move_gripper(300)
    movel(p["down"],       vel=VELOCITY, acc=ACC)
    mwait()
    gripper.close_gripper()
    movel(p["down_x"],     vel=VELOCITY, acc=ACC)
    mwait()
    gripper.move_gripper(300)
    movel(p["up_x"],       vel=VELOCITY, acc=ACC)
    mwait()
    movel(p["see_pill"],   vel=VELOCITY, acc=ACC)
    mwait()
    return p


# === 2단계: 배치 ===
def run_sequence_place(base, dz, dx, dy, dx_close, dz_close):
    current_move(2, 50)
    mwait()
    current_move(0, -100)
    mwait()
    p = make_variants(base, dz, dx, dy, dx_close, dz_close)
    movel(p["close_pose"], vel=VELOCITY, acc=ACC)
    mwait()
    movel(p["close"],      vel=VELOCITY, acc=ACC)
    mwait()
    current_move_two(0, -50, 2, 30)
    mwait()
    return p


def current_move(a, b):
    position_list = list(posx(get_current_posx()[0]))
    position_list[a] += b
    modified_position = posx(position_list)
    movel(modified_position, vel=VELOCITY, acc=ACC) 

def current_move_two(a, b, c, d):
    position_list = list(posx(get_current_posx()[0]))
    position_list[a] += b
    position_list[c] += d
    modified_position = posx(position_list)
    movel(modified_position, vel=VELOCITY, acc=ACC) 

def drug_info(name: str):
    data = {
        "zaide": {"pos": "4_1", "group": "list_drug"},
        "penzal": {"pos": "4_2", "group": "list_drug"},
        "tg": {"pos": "4_3", "group": "list_drug"},
        "sky": {"pos": "4_4", "group": "list_drug"},
        "famotidine": {"pos": "box_center", "group": "list_box"},
        "somnifacient": {"pos": "box_right", "group": "list_box"},
        "allergy": {"pos": "box_left", "group": "list_box"}
    }
    return data.get(name)


def execute_from_key(base_key: str, step="pick"):
    sequence_params = {
        "4_1": {"dz": 40.0, "dx": -90.0,  "dy": 0,   "dx_close": 120.0, "dz_close": 30.0},
        "4_2": {"dz": 40.0, "dx": -85.0,  "dy": 0,   "dx_close": 115.0, "dz_close": 30.0},
        "4_3": {"dz": 40.0, "dx": -90.0, "dy": -50, "dx_close": 105.0, "dz_close": 0.0},
        "4_4": {"dz": 40.0, "dx": -90.0, "dy": 50,  "dx_close": 105.0, "dz_close": 0.0},
    }

    if base_key not in sequence_params:
        print(f"[ERROR] {base_key} 는 등록되지 않은 시퀀스입니다.")
        return

    params = sequence_params[base_key]
    if step == "pick":
        run_sequence_pick(base=base_key, **params)
    elif step == "place":
        run_sequence_place(base=base_key, **params)

def into_capsule():
    movel(posx([346.165, 234.503, 45.317, 24.895, -179.953, -70.178]), vel=VELOCITY, acc=ACC)
    mwait()
    gripper.move_gripper(300)
    current_move(2, 20)
    mwait()
    current_move(1, 60)
    mwait()
    current_move(2, -30)
    mwait()
    movel(posx([346.297, 245.098, 39.873, 30.492, 179.998, -64.633]), vel=VELOCITY, acc=ACC)
    mwait()
    # 힘제어로 캡슐 닫기
    force_start(100, -15)
    while not check_force_condition(DR_AXIS_Z, max=7):
            pass
    force_end()

    # 그리퍼가 잡기 쉽게 자세 잡기
    movel(posx([346.123, 254.796, 45.364, 23.685, -179.958, -71.415]), vel=VELOCITY, acc=ACC)
    mwait()
    movej(posj([35.367, 12.69, 99.83, -0.059, 67.513, -149.557]), vel=VELOCITY, acc=ACC)
    mwait()
    # 캡슐 잡아서 올리기
    gripper.move_gripper(450)
    current_move(2, -30)
    mwait()
    current_move(1, -20)
    mwait()
    gripper.move_gripper(400)
    current_move(2, 50)
    mwait()
