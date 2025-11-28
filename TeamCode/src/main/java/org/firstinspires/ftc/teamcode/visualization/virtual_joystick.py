
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)

# 中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 仿真参数
T = 6.0
dt = 0.01
t = np.arange(0, T, dt)


# 操手输入：恒定命令（如0.5），模拟持续旋转
joystick = np.ones_like(t) * 0.5
joystick += 0.01 * np.random.randn(len(t))  # 轻微手抖

# 理论角度（无扰动、无虚拟摇杆理想情况）
angle_ref = np.cumsum(joystick) * dt

# 扰动窗口
disturb_start, disturb_end = 2.5, 3.5
disturb = np.zeros_like(t)
disturb_idx = (t > disturb_start) & (t < disturb_end)
# 负脉冲扰动，模拟突然被外力推了一下
disturb[disturb_idx] = -1.0 * np.exp(-4 * (t[disturb_idx] - disturb_start))

# 方案A：无虚拟摇杆（仅靠内环慢慢恢复）
angle_A = np.zeros_like(t)
rate_A = np.zeros_like(t)
tau_A = 0.18  # 恢复慢
for i in range(1, len(t)):
    # 角速度 = 操手命令 + 扰动 - 当前角速度 / tau
    rate_A[i] = rate_A[i-1] + (dt / tau_A) * (joystick[i-1] + disturb[i-1] - rate_A[i-1])
    angle_A[i] = angle_A[i-1] + rate_A[i-1] * dt


# 方案B：有虚拟摇杆（检测到角度偏差自动生成反向命令，目标为理论角度）
angle_B = np.zeros_like(t)
rate_B = np.zeros_like(t)
tau_B = 0.10  # 更快恢复
for i in range(1, len(t)):
    # 目标为理论角度（即理想轨迹）
    angle_error = angle_ref[i-1] - angle_B[i-1]
    feedback = 0.0
    if disturb_idx[i] and abs(angle_error) > 0.005:
        feedback = 5 * angle_error  # 更强力拉回目标
    # 角速度 = 操手命令 + 扰动 + 虚拟摇杆自动命令 - 当前角速度 / tau
    rate_B[i] = rate_B[i-1] + (dt / tau_B) * (joystick[i-1] + disturb[i-1] + feedback - rate_B[i-1])
    angle_B[i] = angle_B[i-1] + rate_B[i-1] * dt

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)


# 角度曲线
ax1.plot(t, angle_A, label="无虚拟摇杆：角度", color='#1f77b4', linewidth=1.6)
ax1.plot(t, angle_B, label="虚拟摇杆：角度", color='#ff7f0e', linewidth=1.6)
ax1.plot(t, angle_ref, label="理论角度（参考线）", color='green', linestyle=':', linewidth=1.5)
ax1.axvspan(disturb_start, disturb_end, color='gray', alpha=0.13)
ax1.set_ylabel("角度 (rad)")
ax1.set_title("扰动下角度控制：虚拟摇杆可自动抑制扰动，快速回归目标")
ax1.legend(loc="upper left")
ax1.grid(alpha=0.3)

# 角速度曲线
ax2.plot(t, rate_A, label="无虚拟摇杆：角速度", color='#1f77b4', linestyle='--')
ax2.plot(t, rate_B, label="虚拟摇杆：角速度", color='#ff7f0e', linestyle='--')
ax2.plot(t, joystick, label="操手命令", color='gray', alpha=0.7)
ax2.axvspan(disturb_start, disturb_end, color='gray', alpha=0.13)
ax2.set_xlabel("时间 (s)")
ax2.set_ylabel("角速度 (rad/s)")
ax2.set_title("扰动窗口内，虚拟摇杆自动生成反向命令，快速恢复")
ax2.legend(loc="upper left")
ax2.grid(alpha=0.3)

plt.tight_layout()
plt.show()
