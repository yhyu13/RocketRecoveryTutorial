import numpy as np
from simple_pid import PID
import matplotlib.pyplot as plt

# kp的探讨
iterations = 5  # 迭代次数
simu_steps = 2000  # 仿真步数
h_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于存储高度
v_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于存储速度
ctrl_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于存储控制量
m_array = np.ndarray([iterations, simu_steps], dtype=float)  # 定义一个二维数组用于存储质量
dt = 0.05  # 时间步长，单位为秒
g = 9.82  # 重力加速度，单位为米/秒²
max_thrust = 1e5  # 最大推力，单位为牛顿
alpha = -0.2e3  # 如果最大推力，则速度变化率（每秒）
target = 1000  # 目标高度，单位为米

for turns in range(iterations):
    # 初始化PID控制器，Kp为0.02+0.01*turns，Ki=0.0001，Kd=0.2，setpoint=target，output_limits=(0, 1)，differential_on_measurement=False
    controller = PID(Kp=0.02 + 0.01 * turns, Ki=0.0001, Kd=0.2, setpoint=target, output_limits=(0, 1),
                     differential_on_measurement=False)
    controller.reset()

    mass = 5e3 + 1  # 初始质量为5000克
    h = 0  # 初始高度为0
    v = 0  # 初始速度为0
    for i in range(simu_steps):
        # 计算控制量
        ctrl = controller(h, dt)
        # 计算加速度
        F = max_thrust * ctrl - mass * g  # 计算受力
        '''
            update
            date: 20231005
            author: MrGEFORCE
            [English]Remember to plus dt when updates the mass. I forgot this in the old version, and only
            the code for the descent model was correct.
            The defined alpha is the fuel consumed in 1 second, of course it needs to be multiplied by dt.
            The discussion of ki and kd, as well as the simulation of white noise, have also been changed,
            only here gives the comments to explain these changes.
            [Chinese]质量的更新记得带上dt，之前忘了，原本只有下降模型的代码是正确的。
            因为定义的alpha是一秒消耗的燃料，所以当然要乘上dt。对ki和kd的讨论以及白噪声的仿真都改了，但注释仅在此处提示一下。
        '''
        mass += alpha * ctrl * dt  # 更新质量
        # 计算加速度
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
    string.append("kp=" + str(0.02 + 0.01 * i)[:4])
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