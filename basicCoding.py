
import numpy as np
import matplotlib.pyplot as plt

# 创建一个3行500列的浮点型数组，作为信号数据的存储空间
signal = np.ndarray([3, 500], dtype=float)

# 生成一个从0到π的等差数组，步长为π/500
x = np.arange(0, np.pi, np.pi / 500)

# 计算信号数据，分别使用不同的频率
signal[0] = np.cos(2 * np.pi * x * 3)
signal[1] = np.cos(2 * np.pi * x * 1)
signal[2] = np.cos(2 * np.pi * x * 7)

# 创建第一个图表
plt.figure(0)

# 绘制三个信号的曲线
for i in range(3):
    plt.plot(signal[i])

# 添加图表标题
plt.title("sine wave")

# 添加图例
plt.legend(["w=3", "w=1", "w=7"])

# 显示图表
plt.show()