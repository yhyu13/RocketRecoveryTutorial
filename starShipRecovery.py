import krpc
import time
import numpy as np
from simple_pid import PID

# 连接KRPC服务器
conn = krpc.connect(
    name='recovery',
    address='127.0.0.1',
    rpc_port=50000,
    stream_port=50001
)
space_center = conn.space_center
vessel = space_center.active_vessel

# 判断是否使用自动控制
USING_AP = False
# 获取根部件
root = vessel.parts.root
# 创建栈，用于遍历部件
stack = [(root, 0)]
# 创建KS25引擎部件列表
KS25Engine_parts = []
# 遍历部件，将KS25引擎部件添加到列表中
while stack:
    part, depth = stack.pop()
    if part.title == "S3 KS-25“矢量”液体燃料引擎":
        KS25Engine_parts.append(part)
    for child in part.children:
        stack.append((child, depth+1))


# 定义PID控制器
# 定义一个名为 PIDUsingV 的类
class PIDUsingV:
   # 定义类的构造函数，用于初始化类的属性
   def __init__(self):
       # 初始化 kp、ki 和 kd 的值为 1、0 和 0
       self.kp = 1
       self.ki = 0
       self.kd = 0

       # 初始化最大值和最小值为 1 和 -1
       self.maximum = 1
       self.minimum = -1

       # 记录上次更新时间
       self.prev_t = time.time()
       # 初始化积分值为 0
       self.integral = 0

   # 定义一个名为 init 的方法，用于初始化类的属性
   def init(self, kp, ki, kd, upper, lower):
       # 设置 kp、ki 和 kd 的值
       self.kp = kp
       self.ki = ki
       self.kd = kd
       # 设置最大值和最小值的值
       self.maximum = upper
       self.minimum = lower

   # 定义一个名为 update 的方法，用于更新 PID 控制器的输出值
   def update(self, err, v):
       # 计算时间差
       dt = time.time() - self.prev_t
       # 更新上次更新时间
       self.prev_t = time.time()
       # 计算积分值
       self.integral += err * self.ki * dt
       # 限制积分值在最大值和最小值之间
       self.integral = max(min(self.integral, self.maximum), self.minimum)
       # 计算输出值
       return max(min(err * self.kp + self.integral + v * self.kd, self.maximum), self.minimum)



# 设置引擎偏转
def set_engine_gimbal(lists: list, gim: float):
    gim = max(min(gim, 1), 0)
    for item in lists:
        item.engine.gimbal_limit = gim


# 参考旋转
def reference_rotate():
    lon = vessel.flight().longitude
    ref = space_center.ReferenceFrame.create_relative(vessel.orbit.body.reference_frame, rotation=(
        0, np.sin(-lon / 2. * np.pi / 180), 0, np.cos(-lon / 2. * np.pi / 180)))
    return ref


# 控制
def steering(pitch_bias: float, ref: space_center.ReferenceFrame):
    v = vessel.flight(ref).velocity
    retro_steering = v / np.linalg.norm(v)

    pitch_controller.setpoint = - np.arcsin(retro_steering[0]) / np.pi * 180 + pitch_bias
    heading_controller.setpoint = - np.arctan(retro_steering[1] / retro_steering[2]) / np.pi * 180 + 270

    vessel.control.pitch = pitch_controller(vessel.flight(ref).pitch)
    vessel.control.yaw = heading_controller(vessel.flight(ref).heading)
    vessel.control.roll = roll_controller(vessel.flight(ref).roll)


# 控制AP
def steering_ap(pitch_bias: float, ref: space_center.ReferenceFrame):
    vessel.auto_pilot.reference_frame = ref
    v = vessel.flight(ref).velocity
    retro_steering = v / np.linalg.norm(v)
    target_pitch = - np.arcsin(retro_steering[0]) / np.pi * 180 + pitch_bias
    target_heading = - np.arctan(retro_steering[1] / retro_steering[2]) / np.pi * 180 + 270
    vessel.auto_pilot.target_pitch_and_heading(target_pitch, target_heading)


# 打印信息
print("auto landing started")
vessel.control.gear = False
vessel.control.sas = False
vessel.control.rcs = True
vessel.control.toggle_action_group(8)  # grid fin deploy

# 判断是否使用自动控制
if USING_AP:
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = reference_rotate()
    vessel.auto_pilot.target_roll = 0
else:
    # 初始化PID控制器
    pitch_controller = PID(Kp=0.1, Ki=0.01, Kd=0.4, output_limits=(-1, 1), differential_on_measurement=False)
    heading_controller = PID(Kp=0.1, Ki=0.01, Kd=0.4, output_limits=(-1, 1), differential_on_measurement=False)
    roll_controller = PID(Kp=0.02, Ki=0, Kd=0.01, output_limits=(-1, 1), differential_on_measurement=False, setpoint=0)

# 初始化最终油门控制器
final_throttle = PIDUsingV()
final_throttle.init(kp=0.01, ki=0, kd=0.03, upper=0.8, lower=0)

# 设置引擎偏转
set_engine_gimbal(KS25Engine_parts, 0.2)

# 循环，直到高度小于1000
while vessel.flight().mean_altitude > 1000:
    # 获取参考旋转
    ref = reference_rotate()
    # 判断是否使用自动控制
    if USING_AP:
        # 使用自动控制
        steering_ap(-10, ref)
    else:
        # 控制
        steering(-10, ref)

    # 判断高度，如果高度小于26000，且速度大于1300，则设置油门
    if vessel.flight().mean_altitude < 26000 and vessel.flight(ref).speed > 1300:
        vessel.control.throttle = 0.33
    else:
        # 否则，油门设置为0
        vessel.control.throttle = 0
    # 等待0.05秒
    time.sleep(0.05)

# 打印信息
print("final stage")
# 如果不是自动控制，则初始化PID控制器
if not USING_AP:
    pitch_controller.tunings = (0.4, 0.01, 1)
    heading_controller.tunings = (0.4, 0.01, 1)
# 设置 Gear Deploy 为 True
bGearDeploy = True
# 循环，直到高度小于14
while vessel.flight().surface_altitude > 14:
    # 获取参考旋转
    ref = reference_rotate()
    # 判断是否使用自动控制
    vessel.control.throttle = final_throttle.update(13 - vessel.flight(ref).surface_altitude, -vessel.flight(ref).vertical_speed)
    # 判断速度，如果速度大于50，则控制
    if np.linalg.norm(vessel.flight(ref).velocity[1:]) > 50:
        if USING_AP:
            steering_ap(-10, ref)
        else:
            steering(-10, ref)
    else:
        if USING_AP:
            steering_ap(0, ref)
        else:
            steering(0, ref)

    # 判断高度，如果高度小于250，则设置Gear Deploy为False，并设置油门为1
    if bGearDeploy and vessel.flight().surface_altitude < 250:
        bGearDeploy = False
        vessel.control.gear = True
    # 等待0.01秒
    time.sleep(0.01)

# 设置油门为0
vessel.control.throttle = 0
# 设置自动控制为False
vessel.control.rcs = False
# 打印信息
print("landing complete")