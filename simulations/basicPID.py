import numpy as np
from simple_pid import PID
import matplotlib.pyplot as plt

# 火箭悬停问题物理场景仿真

# 定义时间步长
dt = 0.05  # s
# 定义质量
mass = 5e3 + 1  # unit:kg
# 定义重力加速度
g = 9.82  # m/s^2
# 定义最大推力
max_thrust = 1e5  # N, twr max = 2
# 定义加速度变化率
alpha = -0.02e3  # if max thrust, rate = . t/s

# 定义目标高度
target = 1000
# 定义初始高度
h = 0
# 定义初始速度
v = 0

# 定义PID控制器
controller = PID(Kp=0.05, Ki=0.0001, Kd=0.2, setpoint=target, output_limits=(0, 1), differential_on_measurement=False)

# 定义模拟步数
simu_steps = 2000
# 定义高度数组
h_array = np.ndarray([simu_steps], dtype=float)
# 定义速度数组
v_array = np.ndarray([simu_steps], dtype=float)
# 定义控制数组
ctrl_array = np.ndarray([simu_steps], dtype=float)
# 定义质量数组
m_array = np.ndarray([simu_steps], dtype=float)

# 开始模拟
for i in range(simu_steps):
    # 计算控制量
    ctrl = controller(h, dt)
    # 计算推力
    F = max_thrust * ctrl - mass * g
    # 计算质量变化量
    mass += alpha * ctrl
    # 计算加速度
    acc = F / mass
    # 计算速度变化量
    v += acc * dt
    # 计算高度变化量
    h += v * dt

    # 将模拟结果存入数组
    h_array[i] = h
    v_array[i] = v
    ctrl_array[i] = ctrl
    m_array[i] = mass
    # 如果质量小于0，则将模拟结果置为0
    if mass < 0:
        h_array[i:] = 0
        v_array[i:] = 0
        ctrl_array[i:] = 0
        m_array[i:] = 0
        break


# 绘制模拟结果
f, ax = plt.subplots(4, 1)
plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=1)

plt.subplot(4, 1, 1)
plt.plot(h_array)
plt.title("height(m)")

plt.subplot(4, 1, 2)
plt.plot(v_array)
plt.title("velocity(m/s)")

plt.subplot(4, 1, 3)
plt.plot(ctrl_array)
plt.title("control")

plt.subplot(4, 1, 4)
plt.plot(m_array/1e3)
plt.title("mass(t)")

plt.show()