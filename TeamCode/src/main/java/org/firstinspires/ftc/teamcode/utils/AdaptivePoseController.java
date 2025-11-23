package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * AdaptivePoseController: 自适应 PID/PD 控制平移 + 旋转。
 * 单位说明:
 *  - 位置: 厘米 (cm)
 *  - 角度: 弧度 (rad)
 *  - 输出: 归一化指令 (dimensionless) ∈ [-1,1]
 * 约定:
 *  - xCmd 前后: 机器人前为正
 *  - yCmd 左右: 机器人左为正 (已从“右为正”改为“左为正”)
 *  - rotateCmd 逆时针为正
 */
public class AdaptivePoseController {

    public static class Output {
        public final double xCmd;      // 前后(机器人前为+), dimensionless
        public final double yCmd;      // 左右(机器人左为+), dimensionless (改)
        public final double rotateCmd; // CCW+, dimensionless
        public final double dxCm;      // 剩余误差 X (cm)
        public final double dyCm;      // 剩余误差 Y (cm)
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

    // ---------- 公共死区 / 终点判定参数 ----------
    private double positionDeadbandCm = 2;          // 到位距离判定 (cm)
    private double headingDeadbandRad = Math.toRadians(5); // 到位角度判定 (rad)
    private double minTransCmd = 0.03;                // 平移最小输出 (克服静摩擦)
    private double transZeroDeadband = 0.01;          // 平移零输出死区
    private final double maxCmd = 1.0;                      // 输出限幅

    // ---------- 自适应平移 PID 参数 ----------
    private double transFarP = 0.12;
    private double transNearP = 0.03;
    private double transFarDistanceCm = 50.0;
    private double transNearDistanceCm = 5.0;
    private double transI = 0.0;      // 积分增益 (可选)
    private double transD = 0.0;      // 导数增益 (可选)
    private double transMaxI = 0.1;   // 积分限幅 (dimensionless)

    // ---------- 自适应旋转 PD 参数 ----------
    private double rotFarP = 0.8;
    private double rotNearP = 1.2;
    private double rotFarErrorRad = Math.toRadians(40);
    private double rotNearErrorRad = Math.toRadians(5);
    private double rotD = 0.05;       // 角度导数增益

    // ---------- 状态缓存 ----------
    private double transIAccum = 0.0;
    private double prevDistanceErr = 0.0; // cm
    private double prevHeadingErr = 0.0;  // rad
    private boolean firstAdaptive = true;
    // 自动时间戳缓存（纳秒）
    private long lastTimestampNanos = 0L;
    private static final double DEFAULT_DT_SEC = 1e-3;

    // ---------- Setters ----------
    public void setPositionDeadbandCm(double v){ positionDeadbandCm = v; }
    public void setHeadingDeadbandDeg(double deg){ headingDeadbandRad = Math.toRadians(deg); }
    public void setMinTransCmd(double v){ minTransCmd = v; }
    public void setTransZeroDeadband(double v){ transZeroDeadband = v; }
    public void setTranslationIntegralLimit(double maxI){ this.transMaxI = maxI; }
    public void setAdaptiveTranslationGains(double nearP, double farP, double nearDistCm, double farDistCm, double iGain, double dGain){
        this.transNearP = nearP; this.transFarP = farP; this.transNearDistanceCm = nearDistCm; this.transFarDistanceCm = farDistCm; this.transI = iGain; this.transD = dGain;
    }
    public void setAdaptiveRotationGains(double nearP, double farP, double nearErrDeg, double farErrDeg, double dGain){
        this.rotNearP = nearP; this.rotFarP = farP; this.rotNearErrorRad = Math.toRadians(nearErrDeg); this.rotFarErrorRad = Math.toRadians(farErrDeg); this.rotD = dGain;
    }

    public void resetAdaptive(){
        transIAccum = 0.0;
        prevDistanceErr = 0.0;
        prevHeadingErr = 0.0;
        firstAdaptive = true;
        lastTimestampNanos = 0L;
    }

    // ---------- Helpers ----------
    private double clamp(double v){ return Math.max(-maxCmd, Math.min(maxCmd, v)); }
    private double normalizeAngle(double rad){
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
    private double applyDeadbandAndMin(double v){
        if (Math.abs(v) < transZeroDeadband) return 0.0;
        double sign = Math.signum(v);
        double mag = Math.abs(v);
        if (mag < minTransCmd) mag = minTransCmd;
        return sign * mag;
    }
    private double interpolate(double value, double low, double high, double gainLow, double gainHigh){
        if (value <= low) return gainLow;
        if (value >= high) return gainHigh;
        double t = (value - low)/(high - low);
        return gainLow + t * (gainHigh - gainLow);
    }

    /**
     * 计算自适应 PID 输出 (不直接驱动电机)。
     * @param targetXcm 目标 X (cm)
     * @param targetYcm 目标 Y (cm)
     * @param targetHeadingRad 目标航向 (rad)
     * @param data 里程计数据
     * @param dtSec 本次循环时间间隔 (秒)，用于积分/导数缩放。若未知可用估计值(例如 0.02)。
     */
    public Output computeAdaptive(double targetXcm, double targetYcm, double targetHeadingRad, @NonNull OdometerData data, double dtSec){
        double currXcm = data.getRobotPosition().getX(DistanceUnit.CM);
        double currYcm = data.getRobotPosition().getY(DistanceUnit.CM);
        double currHeadingRad = data.getHeadingRadians();

        double dx = targetXcm - currXcm;
        double dy = targetYcm - currYcm;
        double distanceErr = Math.hypot(dx, dy);
        double dTheta = normalizeAngle(targetHeadingRad - currHeadingRad);

        boolean atPos = distanceErr <= positionDeadbandCm;
        boolean atHead = Math.abs(dTheta) <= headingDeadbandRad;

        // 平移自适应 P
        double pGain = interpolate(distanceErr, transNearDistanceCm, transFarDistanceCm, transNearP, transFarP);
        double translationMag = pGain * distanceErr;

        // 积分 (仅在接近目标但未到位)
        if (!atPos && distanceErr <= transNearDistanceCm * 2.0 && transI > 0){
            transIAccum += transI * distanceErr * dtSec;
            if (transIAccum > transMaxI) transIAccum = transMaxI; else if (transIAccum < -transMaxI) transIAccum = -transMaxI;
        }
        // 导数
        double dDist = 0.0;
        if (!firstAdaptive && dtSec > 1e-6 && transD > 0){
            dDist = (distanceErr - prevDistanceErr)/dtSec;
        }
        translationMag += transIAccum - transD * dDist;

        double transCmdScale = clamp(translationMag);
        double rawForward = 0.0;
        double rawLateral = 0.0;
        if (!atPos && distanceErr > 1e-6){
            rawForward = (dy / distanceErr) * transCmdScale; // 前后 = 场地Y方向
            rawLateral = -(dx / distanceErr) * transCmdScale;
        }
        rawForward = applyDeadbandAndMin(rawForward);
        rawLateral = applyDeadbandAndMin(rawLateral);
        if (atPos){ rawForward = 0.0; rawLateral = 0.0; }

        // 旋转自适应 PD
        double absThetaErr = Math.abs(dTheta);
        double rotPGain = interpolate(absThetaErr, rotNearErrorRad, rotFarErrorRad, rotNearP, rotFarP);
        double headingRate = 0.0;
        if (!firstAdaptive && dtSec > 1e-6){
            headingRate = (dTheta - prevHeadingErr)/dtSec;
        }
        double rotCmd = rotPGain * dTheta - rotD * headingRate;
        rotCmd = clamp(rotCmd);
        if (atHead) rotCmd = 0.0;

        prevDistanceErr = distanceErr;
        prevHeadingErr = dTheta;
        firstAdaptive = false;

        return new Output(rawForward, rawLateral, rotCmd, dx, dy, dTheta, atPos, atHead);
    }

    /**
     * 自动获取时间间隔的自适应PID接口。无需手动传dtSec，适合一般用法。
     * @param targetXcm 目标X (cm)
     * @param targetYcm 目标Y (cm)
     * @param targetHeadingRad 目标航向 (rad)
     * @param data 里程计
     * @return Output
     */
    public Output computeAdaptive(double targetXcm, double targetYcm, double targetHeadingRad, @NonNull OdometerData data) {
        long now = System.nanoTime();
        double dtSec;
        if (lastTimestampNanos == 0L) {
            dtSec = DEFAULT_DT_SEC;
        } else {
            dtSec = (now - lastTimestampNanos) / 1e9;
            if (dtSec < 1e-4) dtSec = DEFAULT_DT_SEC; // 防止极小或负值
        }
        lastTimestampNanos = now;
        return computeAdaptive(targetXcm, targetYcm, targetHeadingRad, data, dtSec);
    }
}
