package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        public Output(double xCmd, double yCmd, double rotateCmd,
                      double dxCm, double dyCm, double dThetaRad,
                      boolean atPosition, boolean atHeading) {
            this.xCmd = xCmd;
            this.yCmd = yCmd;
            this.rotateCmd = rotateCmd;
            this.dxCm = dxCm;
            this.dyCm = dyCm;
            this.dThetaRad = dThetaRad;
            this.atPosition = atPosition;
            this.atHeading = atHeading;
        }
    }

    // 到位判定阈值
    private double positionDeadbandCm = 2.0;               // cm
    private double headingDeadbandRad = Math.toRadians(5);  // rad

    // 轴向 PID 控制器
    private final AdaptivePIDController xTransPID;     // 控制前后 (以场地坐标的 dy 为误差)
    private final AdaptivePIDController yTransPID;     // 控制左右 (以 -dx 为误差, 左为正)
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

    public void setPositionDeadbandCm(double v){ positionDeadbandCm = v; }
    public void setHeadingDeadbandDeg(double deg){ headingDeadbandRad = Math.toRadians(deg); }

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

        double xCmd, yCmd;
        if (atPos) {
            xCmd = 0.0;
            yCmd = 0.0;
        } else {
            double kpEff = scheduleTransKp(distanceErr);
            // 将误差按调度缩放输入 PID，使 P/I/D 均随距离衰减
            double xRaw = xTransPID.compute(forwardErr * kpEff, dtSec);
            double yRaw = yTransPID.compute(leftErr * kpEff, dtSec);
            xCmd = xRaw; // 若需要额外限幅或近区减速可再加工
            yCmd = yRaw;
        }

        double rotateCmd = atHead ? 0.0 : rotatePID.compute(dTheta, dtSec);

        return new Output(xCmd, yCmd, rotateCmd, dx, dy, dTheta, atPos, atHead);
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
