
import krpc
import time
import numpy as np

# 连接到太空中心
conn = krpc.connect(
    name='launch',
    address='127.0.0.1',
    rpc_port=50000,
    stream_port=50001
)

# 获取太空中心和当前活跃的飞船
space_center = conn.space_center
vessel = space_center.active_vessel

# 设定发射高度和相关参数
target_altitude = 75000
turn_start_altitude = 0
gimbal_changing_altitude = 10000
turn_end_altitude = 30000
turning_theta = 10
alpha = np.log(91 - turning_theta) / (turn_end_altitude - turn_start_altitude)

# 添加一些需要计算的流
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
max_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
mass = conn.add_stream(getattr, vessel, 'mass')

# 关闭一些控制
vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

# 寻找机翼并禁用它们在低空
root = vessel.parts.root
stack = [(root, 0)]
wing_parts = []
KS25Engine_parts = []
while stack:
    part, depth = stack.pop()
    if part.title == "大S型升降副翼1" or part.title == "升降副翼3":
        wing_parts.append(part)
    if part.title == "S3 KS-25“矢量”液体燃料引擎":
        KS25Engine_parts.append(part)
    for child in part.children:
        stack.append((child, depth+1))

# 设置主引擎的偏航
def set_main_engine_gimbal(lists: list, gim: float):
    gim = max(min(gim, 1), 0)
    for item in lists:
        item.engine.gimbal_limit = gim

# 设置控制表面的状态
def set_control_surface_enable(lists: list, status: bool):
    for item in lists:
        item.control_surface.roll_enabled = status
        item.control_surface.pitch_enabled = status
        item.control_surface.yaw_enabled = status

# 禁用机翼并设置主引擎偏航为0.25
set_control_surface_enable(wing_parts, False)
bControlWingsRoll = True
set_main_engine_gimbal(KS25Engine_parts, 0.25)
bGimbalChanged = True

# 设置自动飞行目标为90度偏航和90度俯仰
vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.auto_pilot.engage()

# 启动下一个飞行阶段
vessel.control.activate_next_stage()
print("auto launch started")

# 当离apoapsis还有一定差距时，循环执行以下操作
while apoapsis() < target_altitude * 0.99:
    time.sleep(0.05)  # 休眠0.05秒
    vessel.control.throttle = min(20 * mass() / max_thrust(), 1.0)  # 根据质量与最大推力的比率调整油门

    # 如果主引擎偏航已经改变过
    if bGimbalChanged:
        # 如果高度大于改变引擎偏航的临界值，则打印"change engine gimbal"并禁用变化
        if altitude() > gimbal_changing_altitude:
            print("change engine gimbal")
            bGimbalChanged = False
            set_main_engine_gimbal(KS25Engine_parts, 0.5)

    # 如果机翼的控制已经启用
    if bControlWingsRoll:
        # 如果高度大于转角结束的高度，则打印"enable aerodynamic control"并启用机翼控制
        if altitude() > turn_end_altitude:
            print("enable aerodynamic control")
            bControlWingsRoll = False
            vessel.auto_pilot.target_roll = 0
            set_control_surface_enable(wing_parts, True)

    # 如果转角开始的高度 < 高度 < 转角结束的高度，则设置自动飞行目标为(91 - e^α * (高度 - 转角开始高度))度偏航和90度俯仰
    if turn_start_altitude < altitude() < turn_end_altitude:
        vessel.auto_pilot.target_pitch_and_heading(
            91 - np.exp(alpha) * (altitude() - turn_start_altitude), 90)

    # 如果高度 > 转角结束的高度，则设置自动飞行目标为turning_theta度偏航和90度俯仰
    if altitude() > turn_end_altitude:
        vessel.auto_pilot.target_pitch_and_heading(turning_theta, 90)

    # 将油门设置为0.2
    vessel.control.throttle = 0.2

# 当apoapsis还有0.5%的差距时，循环执行以下操作
while apoapsis() < target_altitude * 1.002:
    time.sleep(0.5)

# 将油门设置为0
vessel.control.throttle = 0
print('apoapsis reached')
vessel.auto_pilot.target_pitch_and_heading(0, 90)

# 当apoapsis的计算时间大于30秒时，循环执行以下操作
while time_to_apoapsis() > 30:
    time.sleep(0.5)

# 将油门设置为1
vessel.control.throttle = 1

# 更新日期
'''
    update
    date: 20231005
    author: MrGEFORCE
    [English]In this starship launch script, here the surface speed before separation is an empirical
    value that may not be effective under different ascent trajectories. We may improve it later.
    The current code needs to be corrected based on the payload and actual ascent trajectory, 
    and the given 1600 does not guarantee the landing trajectory of the booster will exactly fall
    on the eastern continent.
    [Chinese]在星舰的发射代码中，此处的分离前速度是一个经验值，在不同的上升轨迹下可能并不通用，之后可以想办法进行改进，
    当前版本需要根据航天器的载荷和实际上升情况来确定，这个给定的1600并不能100%保证助推器的降落轨迹正好落在东边的大陆上。
'''
while vessel.flight(vessel.orbit.body.orbital_reference_frame).speed < 1600:
    time.sleep(0.02)

# 将油门设置为0
time.sleep(0.5)
vessel.control.throttle = 0
time.sleep(0.5)

# 当高度 < 70001时，循环执行以下操作
while altitude() < 70001:
    time.sleep(0.5)

# 执行动作9
vessel.control.toggle_action_group(9)
vessel.control.activate_next_stage()  # 分离

print('launch finished')  # 打印"launch finished"