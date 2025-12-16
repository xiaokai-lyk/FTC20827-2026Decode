# Fixed Python code to draw a single-figure illustrative schematic that demonstrates advantages of a dual-loop PID chassis.
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

#中文
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文字体为黑体
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示

# Time vector and simple illustrative responses (not a full control simulation)
t = np.linspace(0, 3.0, 400)
vel_dual = 1 - np.exp(-t/0.25)
vel_single = 1 - np.exp(-t/0.6) + 0.05 * np.sin(12 * t) * np.exp(-t*1.5)

pos_dual = np.cumsum(vel_dual) * (t[1]-t[0])
pos_single = np.cumsum(vel_single) * (t[1]-t[0])


# 只保留响应曲线，居中显示
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(t, vel_dual, label="速度（双环PID）")
ax.plot(t, vel_single, label="速度（单环PID）")
ax.plot(t, pos_dual / pos_dual.max(), linestyle="--", label="位置（归一化，双环）")
ax.plot(t, pos_single / pos_single.max(), linestyle="--", label="位置（归一化，单环）")
ax.set_xlabel("时间 (s)")
ax.set_ylabel("归一化数值")
ax.set_title("响应对比：双环PID=快+稳，单环易慢且振荡")
ax.legend(loc="lower right", fontsize="small")
# 直观优点注释
ax.text(1.5, 0.85, "双环PID：响应快，超调小，稳态快", color="#1f77b4", fontsize=10)
ax.text(1.5, 0.65, "单环PID：响应慢，易振荡，稳态慢", color="#ff7f0e", fontsize=10)
plt.tight_layout()
plt.show()
