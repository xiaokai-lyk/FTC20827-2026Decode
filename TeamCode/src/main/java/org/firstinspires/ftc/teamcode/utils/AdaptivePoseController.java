package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import lombok.Getter;

/**
 * AdaptivePoseController (refactored): decoupled parameters via three PID controllers.
 * 单位:
 *  - 位置: 厘米 (cm)
 *  - 角度: 弧度 (rad)
 * 输出符号约定:
 *  - xCmd 前后: 机器人前为正
 *  - yCmd 左右: 机器人左为正
 *  - rotateCmd: 逆时针为正
 */
public class AdaptivePoseController {

    public static class Output {
        public final double xCmd;      // 前后(机器人前为+)
        public final double yCmd;      // 左右(机器人左为+)
        public final double rotateCmd; // CCW+
        public final double dxCm;      // 目标-当前 X (cm)
        public final double dyCm;      // 目标-当前 Y (cm)
        public final double dThetaRad; // 航向误差 (rad)
        public final boolean atPosition;
        public final boolean atHeading;
        public final double xRaw;
        public final double yRaw;

        public Output(double xCmd, double yCmd, double rotateCmd,
                      double dxCm, double dyCm, double dThetaRad,
                      boolean atPosition, boolean atHeading, double xRaw, double yRaw) {
            this.xCmd = xCmd;
            this.yCmd = yCmd;
            this.rotateCmd = rotateCmd;
            this.dxCm = dxCm;
            this.dyCm = dyCm;
            this.dThetaRad = dThetaRad;
            this.atPosition = atPosition;
            this.atHeading = atHeading;
            this.xRaw = xRaw;
            this.yRaw = yRaw;
        }
    }

    // 到位判定阈值
    public double positionDeadbandCm = 2.0;               // cm
    public double headingDeadbandRad = Math.toRadians(5);  // rad

    public void resetDeadbands(){
        this.positionDeadbandCm = 2.0;
        this.headingDeadbandRad = Math.toRadians(5);
    }

    // 新增：公开对内部 PID 控制器的访问，以便运行时调参
    // 轴向 PID 控制器
    @Getter
    private final AdaptivePIDController xTransPID;     // 控制前后 (以场地坐标的 dy 为误差)
    @Getter
    private final AdaptivePIDController yTransPID;     // 控制左右 (以 -dx 为误差, 左为正)
    @Getter
    private final AdaptivePIDController rotatePID;     // 控制航向 (弧度误差)

    // 简单距离调度参数（平移用）：远处高增益、近处低增益
    private double transKpNear = 0.03;      // 近距离实际 Kp（整体）
    private double transKpFar = 0.12;       // 远距离实际 Kp（整体）
    private double transNearDistanceCm = 5; // 低增益阈值
    private double transFarDistanceCm = 60; // 高增益阈值

    // 时间戳
    private long lastTimestampNanos = 0L;
    private static final double DEFAULT_DT_SEC = 1e-3;

    /**
     * 以三个轴向 PID 控制器构造。
     */
    public AdaptivePoseController(@NonNull AdaptivePIDController xTransPID,
                                  @NonNull AdaptivePIDController yTransPID,
                                  @NonNull AdaptivePIDController rotatePID) {
        this.xTransPID = xTransPID;
        this.yTransPID = yTransPID;
        this.rotatePID = rotatePID;
    }

    /**
     * 复位内部状态（包括各 PID）。
     */
    public void reset() {
        lastTimestampNanos = 0L;
        xTransPID.reset();
        yTransPID.reset();
        rotatePID.reset();
    }


    // 配置距离调度 (确保 near < far)
    public void setTranslationDistanceSchedule(double kpNear, double kpFar, double nearDistCm, double farDistCm){
        this.transKpNear = kpNear;
        this.transKpFar = kpFar;
        this.transNearDistanceCm = Math.max(1e-6, nearDistCm);
        this.transFarDistanceCm = Math.max(this.transNearDistanceCm + 1e-6, farDistCm);
    }

    private double normalizeAngle(double rad){
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    // 简单线性插值 + smoothstep 平滑过渡
    private double scheduleTransKp(double distanceErr){
        double d = Math.abs(distanceErr);
        if (d <= transNearDistanceCm) return transKpNear;
        if (d >= transFarDistanceCm) return transKpFar;
        double t = (d - transNearDistanceCm)/(transFarDistanceCm - transNearDistanceCm);
        // smoothstep(t) to reduce slope near boundaries
        t = t * t * (3 - 2 * t);
        return transKpNear + (transKpFar - transKpNear) * t;
    }

    /**
     * 计算一次（提供 dt）。
     */
    public Output computeAdaptive(double targetXcm, double targetYcm, double targetHeadingRad,
                                  @NonNull OdometerData data, double dtSec){
        double currXcm = data.getRobotPosition().getX(DistanceUnit.CM);
        double currYcm = data.getRobotPosition().getY(DistanceUnit.CM);
        double currHeadingRad = data.getHeadingRadians();

        double dx = targetXcm - currXcm;     // 目标相对当前: +X 向右
        double dy = targetYcm - currYcm;     // +Y 向前
        double distanceErr = Math.hypot(dx, dy);
        double dTheta = normalizeAngle(targetHeadingRad - currHeadingRad);

        boolean atPos = distanceErr <= positionDeadbandCm;
        boolean atHead = Math.abs(dTheta) <= headingDeadbandRad;

        // 将平面误差投影到机器人指令坐标:
        // xCmd (前后+) 使用 dy 作为误差；
        // yCmd (左+) 使用 -dx 作为误差（dx>0 表示目标在右侧 => 左负）。
        double forwardErr = dy;
        double leftErr = -dx;
        double xRaw, yRaw;

        double xCmd, yCmd;
        if (atPos) {
            xCmd = 0.0;
            yCmd = 0.0;
            xRaw = 0.0;
            yRaw = 0.0;
        } else {
            // 使用真实误差计算 PID，避免被 deadzone 过早置零
            xRaw = xTransPID.compute(forwardErr, dtSec);
            yRaw = yTransPID.compute(leftErr, dtSec);
            // 将距离调度映射为 0.1..1.0 的输出缩放，避免远处/近处输出过小
            double kpEff = scheduleTransKp(distanceErr); // 范围: transKpNear..transKpFar
            double t;
            if (transKpFar > transKpNear + 1e-9) {
                t = (kpEff - transKpNear) / (transKpFar - transKpNear); // 0..1
            } else {
                t = 1.0;
            }
            if (t < 0) t = 0; else if (t > 1) t = 1;
            double scaleOut = 0.1 + 0.7 * t;
            xCmd = xRaw * scaleOut;
            yCmd = yRaw * scaleOut;
        }

        double rotateCmd = atHead ? 0.0 : rotatePID.compute(dTheta, dtSec);

        return new Output(xCmd, yCmd, rotateCmd, dx, dy, dTheta, atPos, atHead, xRaw, yRaw);
    }

    /**
     * 自动 dt 版本。
     */
    public Output computeAdaptive(double targetXcm, double targetYcm, double targetHeadingRad,
                                  @NonNull OdometerData data) {
        long now = System.nanoTime();
        double dtSec;
        if (lastTimestampNanos == 0L) {
            dtSec = DEFAULT_DT_SEC;
        } else {
            dtSec = (now - lastTimestampNanos) / 1e9;
            if (dtSec < 1e-4) dtSec = DEFAULT_DT_SEC;
        }
        lastTimestampNanos = now;
        return computeAdaptive(targetXcm, targetYcm, targetHeadingRad, data, dtSec);
    }
}
