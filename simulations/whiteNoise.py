import numpy as np
from simple_pid import PID
import matplotlib.pyplot as plt

# 白噪声对kd的影响
iterations = 2  # 迭代次数
simu_steps = 2000  # 仿真步数
h_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组，用于存储高度数据
v_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组，用于存储速度数据
ctrl_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组，用于存储控制数据
m_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组，用于存储质量数据

dt = 0.05  # 时间步长，单位：秒
g = 9.82  # 重力加速度，单位：米/秒²
max_thrust = 1e5  # 最大推力，单位：牛顿
alpha = -0.2e3  # 质量变化率，单位：千克/秒
target = 1000  # 目标高度，单位：米

controller = PID(Kp=0.05, Ki=0.0001, Kd=0.2, setpoint=target, output_limits=(0, 1), differential_on_measurement=False)

for turns in range(iterations):
    controller.reset()  # 重置控制器状态
    mass = 5e3 + 1  # 初始质量，单位：千克
    h = 0  # 初始高度，单位：米
    v = 0  # 初始速度，单位：米/秒
    for i in range(simu_steps):
        ctrl = controller(h, dt)  # 获取控制指令
        F = max_thrust * ctrl - mass * g  # 计算受力
        mass += alpha * ctrl * dt  # 更新质量
        acc = F / mass  # 计算加速度
        v += acc * dt  # 更新速度
        h += v * dt  # 更新高度

        # 在这里添加添加白噪声的代码
        if turns == 1:
            h += np.random.random() / 10

        h_array[turns, i] = h  # 存储高度数据
        v_array[turns, i] = v  # 存储速度数据
        ctrl_array[turns, i] = ctrl  # 存储控制数据
        m_array[turns, i] = mass  # 存储质量数据

        # 如果质量小于0，跳出循环
        if mass < 0:
            h_array[turns, i:] = 0
            v_array[turns, i:] = 0
            ctrl_array[turns, i:] = 0
            m_array[turns, i:] = 0
            break


f, ax = plt.subplots(4, 1)
plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=1)
f.suptitle("normal")
plt.subplot(4, 1, 1)
plt.plot(h_array[0])
plt.title("height(m)")

plt.subplot(4, 1, 2)
plt.plot(v_array[0])
plt.title("velocity(m/s)")

plt.subplot(4, 1, 3)
plt.plot(ctrl_array[0])
plt.title("control")

plt.subplot(4, 1, 4)
plt.plot(m_array[0] / 1e3)
plt.title("mass(t)")

f, ax = plt.subplots(4, 1)
plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=1)
f.suptitle("white noise")
plt.subplot(4, 1, 1)
plt.plot(h_array[1])
plt.title("height(m)")

plt.subplot(4, 1, 2)
plt.plot(v_array[1])
plt.title("velocity(m/s)")

plt.subplot(4, 1, 3)
plt.plot(ctrl_array[1])
plt.title("control")

plt.subplot(4, 1, 4)
plt.plot(m_array[1] / 1e3)
plt.title("mass(t)")

plt.show()
