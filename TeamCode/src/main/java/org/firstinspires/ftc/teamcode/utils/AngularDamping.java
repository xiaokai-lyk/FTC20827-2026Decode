package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * AngularDamping 类用于实现机器人的角动量阻尼控制逻辑。
 * 它通过对旋转输入进行动态调整来减少机器人在高速移动时由于惯性导致的过度旋转，
 * 提高操控精度与稳定性。
 */
public class AngularDamping {

    // 滤波后的偏航角速率（单位：rad/s），用于平滑实际测量值
    private double yawRateFiltered = 0.0;

    // 连续处于旋转死区内的帧数计数器，用于动态调节增益系数
    private int deadzoneFrames = 0;

    // 当前使用的自适应增益系数 K，根据运动状态动态变化
    private double currentAdaptiveK = Constants.Damping.ADAPTIVE_K_MIN;

    /**
     * 结果封装类，用于返回经过角阻尼处理后的各项数据
     */
    public static class Result {
        // 经过角阻尼修正后的最终旋转指令输出 [-1, 1]
        public final double rotateCmd;

        // 经过滤波的实际角速度（rad/s）
        public final double yawRateFiltered;

        // 当前应用的自适应增益系数
        public final double adaptiveK;

        // 当前连续处于死区中的帧数
        public final int deadzoneFrames;

        /**
         * 构造函数初始化结果对象
         *
         * @param rotateCmd           最终旋转命令
         * @param yawRateFiltered     滤波后角速度
         * @param adaptiveK           使用的增益系数
         * @param deadzoneFrames      死区持续帧数
         */
        Result(double rotateCmd, double yawRateFiltered, double adaptiveK, int deadzoneFrames) {
            this.rotateCmd = rotateCmd;
            this.yawRateFiltered = yawRateFiltered;
            this.adaptiveK = adaptiveK;
            this.deadzoneFrames = deadzoneFrames;
        }
    }

    /**
     * 主要更新方法，计算并返回经角阻尼算法处理后的旋转指令。
     *
     * @param rotateInput       用户原始旋转输入信号 [-1, 1]
     * @param headingRadians    当前机器人朝向角度（弧度制），当前未使用但保留供后续扩展
     * @param data              来自里程计的数据源，包含实时偏航角速度等信息
     * @param rawX              X方向上的平移输入 [-1, 1]，用于影响增益映射
     * @param rawY              Y方向上的平移输入 [-1, 1]，用于影响增益映射
     * @return                  返回一个包含处理结果的对象
     */
    public Result updateAndApply(double rotateInput, double headingRadians, OdometerData data, double rawX, double rawY) {
        // 获取瞬时偏航角速度（来自硬件传感器）
        double yawRateInstant = data.getYawRate();

        // 应用一阶低通滤波器对瞬时角速度进行平滑处理
        yawRateFiltered = Constants.Damping.YAW_RATE_ALPHA * yawRateInstant
                        + (1 - Constants.Damping.YAW_RATE_ALPHA) * yawRateFiltered;

        // 判断是否进入旋转输入死区范围
        boolean inDeadzone = Math.abs(rotateInput) < Constants.Damping.ROTATE_DEADZONE;

        double rotateCmd; // 最终输出的旋转指令

        if (inDeadzone) { // 如果在死区内，则启用角阻尼补偿机制

            // 增加死区帧计数
            deadzoneFrames++;

            // 计算当前平移速度大小，并限制最大为1.0
            double speedMag = Math.min(1.0, Math.hypot(rawX, rawY));

            // 根据当前速度确定基础增益系数 baseK
            double baseK = Constants.Damping.ADAPTIVE_K_MIN
                         + (Constants.Damping.ADAPTIVE_K_SPEED_MAX - Constants.Damping.ADAPTIVE_K_MIN) * speedMag;

            // 计算增益渐变因子 ramp，随死区停留时间增长而增加，直到达到上限
            double ramp = Math.min(1.0, deadzoneFrames / (double) Constants.Damping.DEADZONE_RAMP_FRAMES);

            // 设置最小渐变比例防止初始阶段无响应
            ramp = Math.max(ramp, Constants.Damping.DEADZONE_RAMP_MIN);

            // 得到当前应使用的自适应增益系数 adaptiveK
            double adaptiveK = baseK * ramp;

            // 若检测到较高的角速度，则进一步放大增益以增强抑制效果
            if (Math.abs(yawRateFiltered) > Constants.Damping.HIGH_RATE_THRESHOLD) {
                adaptiveK *= Constants.Damping.HIGH_RATE_MULTIPLIER;
            }

            // 防止增益超出允许的最大值
            adaptiveK = Math.min(adaptiveK, Constants.Damping.ADAPTIVE_K_MAX);

            // 更新当前增益系数缓存
            currentAdaptiveK = adaptiveK;

            // 只有当角速度大于噪声阈值时才执行阻尼补偿操作
            if (Math.abs(yawRateFiltered) > Constants.Damping.YAW_RATE_NOISE) {
                // 执行角阻尼补偿公式：rotateCmd = rotateInput - adaptiveK * yawRateFiltered
                rotateCmd = rotateInput - adaptiveK * yawRateFiltered;
            } else {
                // 否则不施加补偿力矩，直接归零
                rotateCmd = 0.0;
            }
        } else { // 不在死区范围内时的操作

            // 清除死区帧计数
            deadzoneFrames = 0;

            // 直接传递用户的旋转输入作为输出
            rotateCmd = rotateInput;

            // 对滤波后的角速度做一个衰减处理，避免残留效应干扰下次判断
            yawRateFiltered *= 0.9;

            // 将当前增益重置为最小值
            currentAdaptiveK = Constants.Damping.ADAPTIVE_K_MIN;
        }

        // 保证输出旋转指令不超过设定的最大幅度
        rotateCmd = clamp(rotateCmd, -Constants.Damping.MAX_ROTATE_CMD, Constants.Damping.MAX_ROTATE_CMD);

        // 返回封装好的结果对象
        return new Result(rotateCmd, yawRateFiltered, currentAdaptiveK, deadzoneFrames);
    }

    /**
     * 辅助函数，将数值限定在一个指定区间内
     *
     * @param v   待限幅的原始值
     * @param lo  下界
     * @param hi  上界
     * @return    被约束后的值
     */
    private double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
