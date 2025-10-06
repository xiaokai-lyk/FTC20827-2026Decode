package org.firstinspires.ftc.teamcode.subsystems.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.Contract;

import java.util.function.Supplier;

public class XKMecanumDrive {
    // 控制参数
    public static final double MAX_LINEAR_SPEED = 1.0;    // m/s
    public static final double MAX_ANGULAR_SPEED = 2.0;   // rad/s
    private final double L; // 底盘长度/2
    private final double W; // 底盘宽度/2
    private final PIDController posPidX, posPidY, posPidTheta;
    private final PIDController[] speedPids = new PIDController[4];
    private final FeedforwardController ffController;
    private final DcMotorEx[] motors;

    public XKMecanumDrive(double L, double W,
                         PIDController posPidX, PIDController posPidY, PIDController posPidTheta,
                         PIDController speedPidFL, PIDController speedPidFR, PIDController speedPidRL, PIDController speedPidRR,
                         FeedforwardController ffController, DcMotorEx[] motors) {
        this.L = L;
        this.W = W;
        this.posPidX = posPidX;
        this.posPidY = posPidY;
        this.posPidTheta = posPidTheta;
        this.speedPids[0] = speedPidFL;
        this.speedPids[1] = speedPidFR;
        this.speedPids[2] = speedPidRL;
        this.speedPids[3] = speedPidRR;
        this.ffController = ffController;
        this.motors = motors;
    }

    // 主控制循环，传入目标、传感器、编码器、IMU等接口
    public void update(
            @NonNull Pose2d targetPose,
            @NonNull Supplier<Pose2d> currentPoseSupplier,
            @NonNull Supplier<Velocity2d> currentVelSupplier,
            @NonNull Supplier<Accel2d> currentAccelSupplier,
            @NonNull Supplier<double[]> wheelSpeedsSupplier // [fl, fr, rl, rr]
    ) {
        // 1. 读取传感器数据
        Pose2d currentPose = currentPoseSupplier.get();
        Velocity2d currentVel = currentVelSupplier.get();
        Accel2d currentAccel = currentAccelSupplier.get();
        double[] currentWheelSpeeds = wheelSpeedsSupplier.get();

        // 2. 位置环PID - 计算期望速度
        double errorX = targetPose.x - currentPose.x;
        double errorY = targetPose.y - currentPose.y;
        double errorTheta = angleWrap(targetPose.theta - currentPose.theta);
        double desiredVx = posPidX.calculate(errorX);
        double desiredVy = posPidY.calculate(errorY);
        double desiredOmega = posPidTheta.calculate(errorTheta);
        desiredVx = clamp(desiredVx, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        desiredVy = clamp(desiredVy, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        desiredOmega = clamp(desiredOmega, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        // 3. 前馈补偿
        double[] ff = ffController.calculate(currentAccel.ax, currentAccel.ay, currentAccel.alpha, L, W);

        // 4. 运动学逆解 - 计算各轮期望速度
        double[] wheelTargets = kinematicInverse(desiredVx, desiredVy, desiredOmega, L, W);

        // 5. 速度环PID + 前馈 - 计算电机输出
        for (int i = 0; i < 4; i++) {
            double speedError = wheelTargets[i] - currentWheelSpeeds[i];
            double pidOut = speedPids[i].calculate(speedError);
            double pwm = pidOut + ff[i];
            pwm = clamp(pwm, -1.0, 1.0);
            motors[i].setPower(pwm);
        }
    }

    // 运动学逆解
    @NonNull
    @Contract(value = "_, _, _, _, _ -> new", pure = true)
    public static double[] kinematicInverse(double vx, double vy, double omega, double L, double W) {
        double r = L + W;
        double w_fl = vx - vy - r * omega;
        double w_fr = vx + vy + r * omega;
        double w_rl = vx + vy - r * omega;
        double w_rr = vx - vy + r * omega;
        return new double[]{w_fl, w_fr, w_rl, w_rr};
    }

    // 角度标准化 (-π, π]
    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // 数值限幅
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // 辅助数据结构
    public static class Pose2d {
        public double x, y, theta;
        public Pose2d(double x, double y, double theta) {
            this.x = x; this.y = y; this.theta = theta;
        }
    }
    public static class Velocity2d {
        public double vx, vy, omega;
        public Velocity2d(double vx, double vy, double omega) {
            this.vx = vx; this.vy = vy; this.omega = omega;
        }
    }
    public static class Accel2d {
        public double ax, ay, alpha;
        public Accel2d(double ax, double ay, double alpha) {
            this.ax = ax; this.ay = ay; this.alpha = alpha;
        }
    }
}
