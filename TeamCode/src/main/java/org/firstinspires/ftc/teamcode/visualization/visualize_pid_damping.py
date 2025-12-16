
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
# 中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示

# 模拟参数
time = np.linspace(0, 5, 500)
setpoint = 1.0

# 传统PID响应（固定参数，容易过冲/振荡）
trad_response = setpoint * (1 - np.exp(-2*time) * np.cos(6*time))
trad_damping = 0.1 + 0.2 * np.exp(-time)  # 固定阻尼


# 自适应PID响应（动态增益，快速收敛，轻微超调和波动）
# 这里模拟为更快收敛但有小幅超调和阻尼振荡
adapt_pid_response = setpoint * (1 - 1.08 * np.exp(-4*time) * np.cos(4*time))
adapt_damping = 0.1 + 0.4 * (1 - np.exp(-2*time))  # 随速度自适应

# ----------- 增加扰动恢复过程可视化 -------------
# 在1秒时加入扰动，模拟系统恢复
disturb_time = 1.0
disturb_idx = np.searchsorted(time, disturb_time)
disturb_value = 0.3

# 传统阻尼下扰动后的恢复（简单一阶系统模拟）
trad_recover = np.copy(trad_response)
trad_recover[disturb_idx:] += disturb_value * np.exp(-2*(time[disturb_idx:] - disturb_time)) * np.cos(6*(time[disturb_idx:] - disturb_time))

# 自适应阻尼下扰动后的恢复（更快收敛，无明显振荡）
adapt_recover = np.copy(adapt_pid_response)
adapt_recover[disturb_idx:] += disturb_value * np.exp(-5*(time[disturb_idx:] - disturb_time))

plt.figure(figsize=(12, 5))


# 左图：只展示PID响应对比
plt.subplot(1, 2, 1)
plt.plot(time, trad_response, label='传统PID', linestyle='--', color='tab:blue')
plt.plot(time, adapt_pid_response, label='自适应PID', color='tab:orange')
plt.axhline(setpoint, color='gray', linestyle=':', linewidth=1)
plt.title('目标响应对比（无扰动）')
plt.xlabel('时间 (s)')
plt.ylabel('输出/位置')
plt.legend()
plt.grid(True)

# 右图：只展示扰动恢复下的阻尼效果
plt.subplot(1, 2, 2)
plt.plot(time, trad_recover, label='传统阻尼+扰动恢复', linestyle='-', color='tab:blue')
plt.plot(time, adapt_recover, label='自适应阻尼+扰动恢复', linestyle='-', color='tab:orange')
plt.axhline(setpoint, color='gray', linestyle=':', linewidth=1)
plt.axvline(disturb_time, color='red', linestyle=':', linewidth=1.5)
plt.annotate('扰动发生', xy=(disturb_time, setpoint+0.1), xytext=(disturb_time+0.2, setpoint+0.3),
			 arrowprops=dict(facecolor='red', shrink=0.05), color='red', fontsize=12, fontweight='bold')
plt.title('扰动恢复对比（阻尼效果）')
plt.xlabel('时间 (s)')
plt.ylabel('输出/位置')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.suptitle('自适应PID与自适应阻尼对比传统方案优势示意图', fontsize=14, y=1.05)
plt.show()
