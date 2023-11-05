import numpy as np  # 导入numpy库
from simple_pid import PID  # 导入simple_pid库的PID类
import matplotlib.pyplot as plt  # 导入matplotlib库的pyplot模块

# ki的探讨
iterations = 5  # 循环次数
simu_steps = 2000  # 仿真步数
h_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于记录高度
v_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于记录速度
ctrl_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于记录控制量
m_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于记录质量

dt = 0.05  # 时间步长
g = 9.82  # 重力加速度
max_thrust = 1e5  # 最大推力，单位为牛顿
alpha = -0.2e3  # 如果最大推力，质量变化率 = 0.2克/秒
target = 1000  # 目标高度

for turns in range(iterations):
   controller = PID(Kp=0.05, Ki=0.01 + 0.002 * turns, Kd=0.2, setpoint=target, output_limits=(0, 1),
                    differential_on_measurement=False)  # 创建一个PID控制器，设置参数
   controller.reset()  # 重置控制器

   mass = 5e3 + 1  # 初始质量为5000克
   h = 0  # 初始高度为0
   v = 0  # 初始速度为0
   for i in range(simu_steps):
       ctrl = controller(h, dt)  # 计算控制量
       F = max_thrust * ctrl - mass * g  # 计算受力
       mass += alpha * ctrl * dt  # 更新质量
       acc = F / mass  # 计算加速度
       v += acc * dt  # 更新速度
       h += v * dt  # 更新高度

       h_array[turns, i] = h  # 记录高度数组
       v_array[turns, i] = v  # 记录速度数组
       ctrl_array[turns, i] = ctrl  # 记录控制量数组
       m_array[turns, i] = mass  # 记录质量数组
       if mass < 0:  # 如果质量小于0，跳出循环
           h_array[turns, i:] = 0  # 剩余高度设为0
           v_array[turns, i:] = 0  # 剩余速度设为0
           ctrl_array[turns, i:] = 0  # 剩余控制量设为0
           m_array[turns, i:] = 0  # 剩余质量设为0
           break


f, ax = plt.subplots(4, 1)
plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=1)

plt.subplot(4, 1, 1)
string = []
for i in range(iterations):
    plt.plot(h_array[i])
    string.append("ki=" + str(0.01 + 0.002 * i)[:5])
plt.title("height(m)")
plt.legend(string)

plt.subplot(4, 1, 2)
for i in range(iterations):
    plt.plot(v_array[i])
plt.title("velocity(m/s)")

plt.subplot(4, 1, 3)
for i in range(iterations):
    plt.plot(ctrl_array[i])
plt.title("control")

plt.subplot(4, 1, 4)
for i in range(iterations):
    plt.plot(m_array[i] / 1e3)
plt.title("mass(t)")

plt.show()
