package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * 平移自适应阻尼算法封装：根据输入摇杆和当前机器人残余平移速度，对 x / y 分量进行阻尼。
 * 用法：每帧调用 updateAndApply(...) 传入原始输入与 OdometerData，获得修正后的 x,y。
 */
public class TranslationalDamping {
    // 状态
    private double vxFiltered = 0.0; // 低通后的平移速度 x
    private double vyFiltered = 0.0; // 低通后的平移速度 y
    private int deadzoneFramesX = 0;
    private int deadzoneFramesY = 0;

    public static class Result {
        public final double x;
        public final double y;
        public final double kx;
        public final double ky;
        public final int dzX;
        public final int dzY;
        public final double vxFiltered;
        public final double vyFiltered;
        Result(double x, double y, double kx, double ky, int dzX, int dzY, double vxF, double vyF) {
            this.x = x; this.y = y; this.kx = kx; this.ky = ky; this.dzX = dzX; this.dzY = dzY; this.vxFiltered = vxF; this.vyFiltered = vyF;
        }
    }

    /**
     * 更新阻尼并返回修正后的 x,y 输入。
     * @param rawX 驾驶员原始 x 输入（右为正），归一化 -1..1
     * @param rawY 驾驶员原始 y 输入（前为正），归一化 -1..1
     * @param yawRateFiltered 已有的角速度滤波值（供耦合衰减使用，如果没有传 0）
     * @param data OdometerData（需要 robotVx / robotVy）
     */
    public Result updateAndApply(double rawX, double rawY, double yawRateFiltered, OdometerData data) {
        double vxInstant = data.getRobotVx();
        double vyInstant = data.getRobotVy();
        vxFiltered = Constants.Damping.TRANS_ALPHA * vxInstant + (1 - Constants.Damping.TRANS_ALPHA) * vxFiltered;
        vyFiltered = Constants.Damping.TRANS_ALPHA * vyInstant + (1 - Constants.Damping.TRANS_ALPHA) * vyFiltered;

        boolean inDeadzoneX = Math.abs(rawX) < Constants.Damping.TRANS_DEADZONE;
        boolean inDeadzoneY = Math.abs(rawY) < Constants.Damping.TRANS_DEADZONE;
        if (inDeadzoneX) deadzoneFramesX++; else { deadzoneFramesX = 0; vxFiltered *= 0.95; }
        if (inDeadzoneY) deadzoneFramesY++; else { deadzoneFramesY = 0; vyFiltered *= 0.95; }

        double speedMag = Math.min(1.0, Math.hypot(rawX, rawY));
        double baseK = Constants.Damping.ADAPTIVE_K_MIN_T + (Constants.Damping.ADAPTIVE_K_SPEED_MAX_T - Constants.Damping.ADAPTIVE_K_MIN_T) * speedMag;

        double rampX = Math.min(1.0, deadzoneFramesX / (double) Constants.Damping.DEADZONE_RAMP_FRAMES_T);
        rampX = Math.max(rampX, Constants.Damping.DEADZONE_RAMP_MIN_T);
        double rampY = Math.min(1.0, deadzoneFramesY / (double) Constants.Damping.DEADZONE_RAMP_FRAMES_T);
        rampY = Math.max(rampY, Constants.Damping.DEADZONE_RAMP_MIN_T);

        double driftFactorX = Math.abs(vxFiltered) > Constants.Damping.HIGH_DRIFT_THRESHOLD ? Constants.Damping.DRIFT_MULTIPLIER : 1.0;
        double driftFactorY = Math.abs(vyFiltered) > Constants.Damping.HIGH_DRIFT_THRESHOLD ? Constants.Damping.DRIFT_MULTIPLIER : 1.0;

        double rotationCouple = Math.abs(yawRateFiltered) > Constants.Damping.ROTATION_COUPLE_THRESHOLD ? Constants.Damping.ROTATION_COUPLE_FACTOR : 1.0;

        double adaptiveKx = baseK * rampX * driftFactorX * rotationCouple;
        double adaptiveKy = baseK * rampY * driftFactorY * rotationCouple;
        adaptiveKx = clamp(adaptiveKx, Constants.Damping.ADAPTIVE_K_MIN_T, Constants.Damping.ADAPTIVE_K_MAX_T);
        adaptiveKy = clamp(adaptiveKy, Constants.Damping.ADAPTIVE_K_MIN_T, Constants.Damping.ADAPTIVE_K_MAX_T);

        double outX = rawX;
        double outY = rawY;
        if (inDeadzoneX && Math.abs(vxFiltered) > Constants.Damping.TRANS_NOISE) {
            outX = rawX - adaptiveKx * vxFiltered;
        } else if (inDeadzoneX) {
            outX = 0.0;
        }
        if (inDeadzoneY && Math.abs(vyFiltered) > Constants.Damping.TRANS_NOISE) {
            outY = rawY - adaptiveKy * vyFiltered;
        } else if (inDeadzoneY) {
            outY = 0.0;
        }

        outX = clamp(outX, -1.0, 1.0);
        outY = clamp(outY, -1.0, 1.0);

        return new Result(outX, outY, adaptiveKx, adaptiveKy, deadzoneFramesX, deadzoneFramesY, vxFiltered, vyFiltered);
    }

    private double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}
