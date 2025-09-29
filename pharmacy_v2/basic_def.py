# basic_def.py
import time
from DSR_ROBOT2 import (
    release_compliance_ctrl,
    release_force,
    set_digital_output,
    get_digital_input,
    task_compliance_ctrl,
    set_desired_force,
    moveb, mwait,
    DR_FC_MOD_REL, DR_LINE, DR_MV_MOD_ABS, DR_BASE
)
from DR_common2 import posb

OFF, ON = 0, 1

def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        time.sleep(0.5)
        # print(f"Wait for digital input: {sig_num}")

# 그랩 놓는것
def release(tm=0.0):
    # print("set for digital output 0 1 for release")
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    time.sleep(tm)
    # wait_digital_input(2)

def release_soft(tm=0.0):
    # print("set for digital output 1 1 for release")
    set_digital_output(1, ON)
    set_digital_output(2, ON)
    time.sleep(tm)
    # wait_digital_input(2)

# 그랩 하는것
def grip(tm=0.0):
    # print("set for digital output 1 0 for grip")
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    time.sleep(tm)
    # wait_digital_input(1)

def grip_soft(tm=0.0):
    # print("set for digital output 0 0 for grip")
    set_digital_output(2, OFF)
    set_digital_output(1, OFF)
    time.sleep(tm)


# 힘 제어 및 순응 제어 시작
def force_start(task_arg, desired_arg):
    print("force_control_start")
    task_compliance_ctrl(stx=[task_arg, task_arg, task_arg, 100, 100, 100])
    time.sleep(0.1)
    set_desired_force(fd=[0, 0, desired_arg, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    time.sleep(0.1)

# 힘 제어 및 순응 제어 종료
def force_end():
    time.sleep(0.1)
    release_force()
    time.sleep(0.1)
    release_compliance_ctrl()
    print("force_control_end")

# moveb 사용 함수
def place(seg_1, seg_2):
    global VELOCITY, ACC
    seg11 = posb(DR_LINE, seg_1, radius=20)
    seg12 = posb(DR_LINE, seg_2, radius=20)
    down_th = seg_2.copy()
    down_th[2] -= 100
    seg13 = posb(DR_LINE, down_th, radius=20)
    b_list = [seg11, seg12, seg13]
    moveb(b_list, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    mwait()
