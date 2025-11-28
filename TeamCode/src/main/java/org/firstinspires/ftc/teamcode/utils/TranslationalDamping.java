package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * 平移自适应阻尼算法封装：根据输入摇杆和当前机器人残余平移速度，对 x / y 分量进行阻尼。
 * 用法：每帧调用 updateAndApply(...) 传入原始输入与 OdometerData，获得修正后的 x,y。
 */
public class TranslationalDamping {
    // 状态变量

    // 低通滤波后的平移速度 x 分量（机器人坐标系）
    private double vxFiltered = 0.0;

    // 低通滤波后的平移速度 y 分量（机器人坐标系）
    private double vyFiltered = 0.0;

    // X方向连续处于死区内的帧数计数器
    private int deadzoneFramesX = 0;

    // Y方向连续处于死区内的帧数计数器
    private int deadzoneFramesY = 0;

    /**
     * 结果封装类，用于返回经过平移阻尼处理后的各项数据
     */
    public static class Result {
        // 经过阻尼修正后的最终X方向输出 [-1, 1]
        public final double x;

        // 经过阻尼修正后的最终Y方向输出 [-1, 1]
        public final double y;

        // 当前应用于X方向的自适应增益系数
        public final double kx;

        // 当前应用于Y方向的自适应增益系数
        public final double ky;

        // X方向当前连续处于死区中的帧数
        public final int dzX;

        // Y方向当前连续处于死区中的帧数
        public final int dzY;

        // 经过滤波的X方向速度 (m/s)
        public final double vxFiltered;

        // 经过滤波的Y方向速度 (m/s)
        public final double vyFiltered;

        /**
         * 构造函数初始化结果对象
         *
         * @param x           最终X方向输出
         * @param y           最终Y方向输出
         * @param kx          X方向增益系数
         * @param ky          Y方向增益系数
         * @param dzX         X方向死区帧数
         * @param dzY         Y方向死区帧数
         * @param vxF         滤波后X方向速度
         * @param vyF         滤波后Y方向速度
         */
        Result(double x, double y, double kx, double ky, int dzX, int dzY, double vxF, double vyF) {
            this.x = x;
            this.y = y;
            this.kx = kx;
            this.ky = ky;
            this.dzX = dzX;
            this.dzY = dzY;
            this.vxFiltered = vxF;
            this.vyFiltered = vyF;
        }
    }

    /**
     * 主要更新方法，计算并返回经平移阻尼算法处理后的X/Y方向指令。
     *
     * @param rawX              驾驶员原始 X 输入（右为正），归一化范围 -1..1
     * @param rawY              驾驶员原始 Y 输入（前为正），归一化范围 -1..1
     * @param yawRateFiltered   已有的角速度滤波值（供耦合衰减使用，如果没有传 0）
     * @param data              OdometerData（需要 robotVx / robotVy 获取当前速度）
     * @return                  返回一个包含处理结果的对象
     */
    public Result updateAndApply(double rawX, double rawY, double yawRateFiltered, OdometerData data) {
        // 获取瞬时平移速度（来自里程计数据）
        double vxInstant = data.getRobotVx();
        double vyInstant = data.getRobotVy();

        // 应用一阶低通滤波器对瞬时速度进行平滑处理
        vxFiltered = Constants.Damping.TRANS_ALPHA * vxInstant
                   + (1 - Constants.Damping.TRANS_ALPHA) * vxFiltered;
        vyFiltered = Constants.Damping.TRANS_ALPHA * vyInstant
                   + (1 - Constants.Damping.TRANS_ALPHA) * vyFiltered;

        // 判断X/Y方向是否进入平移输入死区范围
        boolean inDeadzoneX = Math.abs(rawX) < Constants.Damping.TRANS_DEADZONE;
        boolean inDeadzoneY = Math.abs(rawY) < Constants.Damping.TRANS_DEADZONE;

        // 处理X方向死区逻辑
        if (inDeadzoneX) {
            deadzoneFramesX++;  // 增加死区帧计数
        } else {
            deadzoneFramesX = 0;  // 清除死区帧计数
            vxFiltered *= 0.95;   // 对滤波速度做衰减处理
        }

        // 处理Y方向死区逻辑
        if (inDeadzoneY) {
            deadzoneFramesY++;  // 增加死区帧计数
        } else {
            deadzoneFramesY = 0;  // 清除死区帧计数
            vyFiltered *= 0.95;   // 对滤波速度做衰减处理
        }

        // 计算当前输入速度大小，并限制最大为1.0
        double speedMag = Math.min(1.0, Math.hypot(rawX, rawY));

        // 根据当前速度确定基础增益系数 baseK
        double baseK = Constants.Damping.ADAPTIVE_K_MIN_T
                     + (Constants.Damping.ADAPTIVE_K_SPEED_MAX_T - Constants.Damping.ADAPTIVE_K_MIN_T) * speedMag;

        // 计算X方向增益渐变因子 rampX，随死区停留时间增长而增加
        double rampX = Math.min(1.0, deadzoneFramesX / (double) Constants.Damping.DEADZONE_RAMP_FRAMES_T);
        rampX = Math.max(rampX, Constants.Damping.DEADZONE_RAMP_MIN_T);

        // 计算Y方向增益渐变因子 rampY
        double rampY = Math.min(1.0, deadzoneFramesY / (double) Constants.Damping.DEADZONE_RAMP_FRAMES_T);
        rampY = Math.max(rampY, Constants.Damping.DEADZONE_RAMP_MIN_T);

        // 检测是否存在高速漂移情况，并设置相应的漂移因子
        double driftFactorX = Math.abs(vxFiltered) > Constants.Damping.HIGH_DRIFT_THRESHOLD
                            ? Constants.Damping.DRIFT_MULTIPLIER : 1.0;
        double driftFactorY = Math.abs(vyFiltered) > Constants.Damping.HIGH_DRIFT_THRESHOLD
                            ? Constants.Damping.DRIFT_MULTIPLIER : 1.0;

        // 检测是否存在旋转耦合影响，并设置相应的耦合因子
        double rotationCouple = Math.abs(yawRateFiltered) > Constants.Damping.ROTATION_COUPLE_THRESHOLD
                              ? Constants.Damping.ROTATION_COUPLE_FACTOR : 1.0;

        // 计算最终应用于X/Y方向的自适应增益系数
        double adaptiveKx = baseK * rampX * driftFactorX * rotationCouple;
        double adaptiveKy = baseK * rampY * driftFactorY * rotationCouple;

        // 限制增益系数在有效范围内
        adaptiveKx = clamp(adaptiveKx, Constants.Damping.ADAPTIVE_K_MIN_T, Constants.Damping.ADAPTIVE_K_MAX_T);
        adaptiveKy = clamp(adaptiveKy, Constants.Damping.ADAPTIVE_K_MIN_T, Constants.Damping.ADAPTIVE_K_MAX_T);

        // 初始化输出值为原始输入值
        double outX = rawX;
        double outY = rawY;

        // 如果X方向在死区且速度超过噪声阈值，则执行阻尼补偿
        if (inDeadzoneX && Math.abs(vxFiltered) > Constants.Damping.TRANS_NOISE) {
            outX = rawX - adaptiveKx * vxFiltered;
        } else if (inDeadzoneX) {
            // 如果在死区但速度很小，则直接置零
            outX = 0.0;
        }

        // 如果Y方向在死区且速度超过噪声阈值，则执行阻尼补偿
        if (inDeadzoneY && Math.abs(vyFiltered) > Constants.Damping.TRANS_NOISE) {
            outY = rawY - adaptiveKy * vyFiltered;
        } else if (inDeadzoneY) {
            // 如果在死区但速度很小，则直接置零
            outY = 0.0;
        }

        // 保证输出值不超过[-1, 1]范围
        outX = clamp(outX, -1.0, 1.0);
        outY = clamp(outY, -1.0, 1.0);

        // 返回封装好的结果对象
        return new Result(outX, outY, adaptiveKx, adaptiveKy, deadzoneFramesX, deadzoneFramesY, vxFiltered, vyFiltered);
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
