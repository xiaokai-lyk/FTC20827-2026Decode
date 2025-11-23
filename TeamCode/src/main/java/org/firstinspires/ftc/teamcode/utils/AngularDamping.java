package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Constants;

public class AngularDamping {
    // 状态（去掉 heading 差分相关）
    private double yawRateFiltered = 0.0;
    private int deadzoneFrames = 0;
    private double currentAdaptiveK = Constants.Damping.ADAPTIVE_K_MIN;

    public static class Result {
        public final double rotateCmd; // 修正后的旋转指令
        public final double yawRateFiltered; // 滤波后的角速度 (rad/s)
        public final double adaptiveK; // 当前增益
        public final int deadzoneFrames; // 死区帧计数
        Result(double rotateCmd, double yawRateFiltered, double adaptiveK, int deadzoneFrames) {
            this.rotateCmd = rotateCmd;
            this.yawRateFiltered = yawRateFiltered;
            this.adaptiveK = adaptiveK;
            this.deadzoneFrames = deadzoneFrames;
        }
    }

    /**
     * 更新角阻尼并返回修正旋转指令（无需时间差分）。
     * @param rotateInput 原始旋转输入 [-1,1]
     * @param headingRadians 当前航向 (rad)（保留参数以便未来扩展，可未使用）
     * @param data OdometerData (若提供 yawRate 则直接用)
     * @param rawX 平移输入 X 用于增益映射 [-1,1]
     * @param rawY 平移输入 Y 用于增益映射 [-1,1]
     */
    public Result updateAndApply(double rotateInput, double headingRadians, OdometerData data, double rawX, double rawY) {
        // 直接获取瞬时角速度
        double yawRateInstant = data.getYawRate();
        // 低通滤波
        yawRateFiltered = Constants.Damping.YAW_RATE_ALPHA * yawRateInstant + (1 - Constants.Damping.YAW_RATE_ALPHA) * yawRateFiltered;

        boolean inDeadzone = Math.abs(rotateInput) < Constants.Damping.ROTATE_DEADZONE;
        double rotateCmd;
        if (inDeadzone) {
            deadzoneFrames++;
            double speedMag = Math.min(1.0, Math.hypot(rawX, rawY));
            double baseK = Constants.Damping.ADAPTIVE_K_MIN + (Constants.Damping.ADAPTIVE_K_SPEED_MAX - Constants.Damping.ADAPTIVE_K_MIN) * speedMag;
            double ramp = Math.min(1.0, deadzoneFrames / (double) Constants.Damping.DEADZONE_RAMP_FRAMES);
            ramp = Math.max(ramp, Constants.Damping.DEADZONE_RAMP_MIN);
            double adaptiveK = baseK * ramp;
            if (Math.abs(yawRateFiltered) > Constants.Damping.HIGH_RATE_THRESHOLD) {
                adaptiveK *= Constants.Damping.HIGH_RATE_MULTIPLIER;
            }
            adaptiveK = Math.min(adaptiveK, Constants.Damping.ADAPTIVE_K_MAX);
            currentAdaptiveK = adaptiveK;
            if (Math.abs(yawRateFiltered) > Constants.Damping.YAW_RATE_NOISE) {
                rotateCmd = rotateInput - adaptiveK * yawRateFiltered;
            } else {
                rotateCmd = 0.0;
            }
        } else {
            deadzoneFrames = 0;
            rotateCmd = rotateInput;
            yawRateFiltered *= 0.9; // 退出死区衰减滤波残留
            currentAdaptiveK = Constants.Damping.ADAPTIVE_K_MIN;
        }
        rotateCmd = clamp(rotateCmd, -Constants.Damping.MAX_ROTATE_CMD, Constants.Damping.MAX_ROTATE_CMD);
        return new Result(rotateCmd, yawRateFiltered, currentAdaptiveK, deadzoneFrames);
    }

    private double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
