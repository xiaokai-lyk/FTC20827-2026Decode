import numpy as np
import matplotlib.pyplot as plt

# 中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 模拟编码器原始数据（含噪声和突变）
np.random.seed(0)
t = np.linspace(0, 10, 200)
true_angle = np.sin(t) * 30  # 理想角度
 # 高频随机噪声
high_freq_noise = np.random.normal(0, 9, size=t.shape) * np.sin(100 * t)
raw_angle = true_angle + high_freq_noise

raw_angle[100:101] += 40

# EMA低通滤波
alpha = 0.3
filtered_angle = np.zeros_like(raw_angle)
filtered_angle[0] = raw_angle[0]
for i in range(1, len(raw_angle)):
    filtered_angle[i] = alpha * raw_angle[i] + (1 - alpha) * filtered_angle[i-1]

plt.figure(figsize=(10,5))
plt.subplot(2, 1, 1)
plt.plot(t, raw_angle, label='原始编码器数据', color='red', alpha=0.6)
plt.plot(t, filtered_angle, label='EMA滤波后', color='blue', linewidth=2)
plt.plot(t, true_angle, label='理想角度', color='green', linestyle='--')
plt.legend()
plt.xlabel('时间')
plt.ylabel('角度 (度)')
plt.title('EMA低通滤波效果演示')
plt.grid(True)

# 残差可视化
residual_raw = raw_angle - true_angle
residual_filtered = filtered_angle - true_angle

plt.subplot(2, 1, 2)
plt.plot(t, residual_raw, label='原始数据残差', color='red', alpha=0.6)
plt.plot(t, residual_filtered, label='EMA滤波后残差', color='blue', linewidth=2)
plt.axhline(0, color='gray', linestyle='--', linewidth=1)
plt.legend()
plt.xlabel('时间')
plt.ylabel('残差 (度)')
plt.title('残差对比')
plt.grid(True)
plt.tight_layout()
plt.show()