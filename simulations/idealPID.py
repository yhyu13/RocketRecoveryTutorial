
import numpy as np  # 导入numpy库，用于数组操作
from simple_pid import PID  # 导入PID库，用于控制器的设计
import matplotlib.pyplot as plt  # 导入绘图库

dt = 2  # 定义时间步长
target = 100  # 定义目标值

# 创建一个PID控制器对象，设置P、I、D参数以及设定值和测量值的差分开关状态
controller = PID(Kp=0.04, Ki=0.001, Kd=0.02, setpoint=target, differential_on_measurement=False, output_limits=(0, 5))

steps = 500  # 定义模拟的步数
value = 0  # 定义初始值
array = np.ndarray([steps], dtype=float)  # 创建一个用于存储模拟结果的数组
target_array = np.ndarray([steps], dtype=float)  # 创建一个用于存储目标值的数组
target_array[:] = target  # 将目标数组的每个元素都设置为目标值

# 进行步态模拟
for i in range(steps):
    value += controller(value, dt)  # 根据当前值和时间步长调用控制器得到控制量
    array[i] = value  # 将当前值保存到结果数组中

plt.figure(0)  # 创建一个新的画布
plt.plot(array)  # 绘制结果数组中的值
plt.plot(target_array)  # 绘制目标数组中的值
plt.show()  # 显示绘制的图像