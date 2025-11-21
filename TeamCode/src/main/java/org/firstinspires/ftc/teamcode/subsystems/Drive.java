package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive {
    public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
    private DcMotor.RunMode currentMode = null;

    public Drive(@NonNull Hardwares hardwares){
        this.mFrontLeft = hardwares.motors.mFrontLeft;
        this.mFrontRight = hardwares.motors.mFrontRight;
        this.mBackLeft = hardwares.motors.mBackLeft;
        this.mBackRight = hardwares.motors.mBackRight;
    }

    public void runWithEncoders(){
        if (currentMode != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            mFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            currentMode = DcMotorEx.RunMode.RUN_USING_ENCODER;
        }
    }

    public void runWithoutEncoders(){
        if (currentMode != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            mFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            currentMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        }
    }

    public void setPower(@NonNull double[] potentials){
        mFrontLeft.setPower(potentials[0]);
        mFrontRight.setPower(potentials[1]);
        mBackLeft.setPower(potentials[2]);
        mBackRight.setPower(potentials[3]);
    }

    public void setVelocity(@NonNull double[] potentials){
        mFrontLeft.setVelocity(potentials[0] * Constants.driveMaxVelocity);
        mFrontRight.setVelocity(potentials[1] * Constants.driveMaxVelocity);
        mBackLeft.setVelocity(potentials[2] * Constants.driveMaxVelocity);
        mBackRight.setVelocity(potentials[3] * Constants.driveMaxVelocity);
    }

    public double[] calculateComponents(double x, double y, double rotate, double headingRadians, double speedCoefficient){
        double rotX = x * Math.cos(-headingRadians) - y * Math.sin(-headingRadians);
        double rotY = x * Math.sin(-headingRadians) + y * Math.cos(-headingRadians);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator * speedCoefficient;
        double backLeftPower = (rotY - rotX + rotate) / denominator * speedCoefficient;
        double frontRightPower = (rotY - rotX - rotate) / denominator * speedCoefficient;
        double backRightPower = (rotY + rotX - rotate) / denominator * speedCoefficient;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }

    public static class TeleOpDriveCommand extends CommandBase {
        private final DoubleSupplier x;
        private final DoubleSupplier rotate;
        private final DoubleSupplier y;
        private final DoubleSupplier speedCoefficient;
        private final Drive drive;
        private final DoubleSupplier heading; // 当前航向（弧度）
        private final BooleanSupplier useEncoders;

        // 角速度阻尼参数 (基础噪声/滤波仍保持)
        private static final double ROTATE_DEADZONE = 0.02; // 摇杆死区
        private static final double YAW_RATE_ALPHA = 0.30; // 一阶低通滤波系数
        private static final double YAW_RATE_NOISE = 0.02; // 小于该阈值视为噪声不补偿
        private static final double MAX_ROTATE_CMD = 1.0; // 输出限幅

        // 自适应K参数
        private static final double ADAPTIVE_K_MIN = 0.00; // 最小阻尼增益
        private static final double ADAPTIVE_K_MAX = 0.25; // 允许的最大阻尼增益（总体）
        private static final double ADAPTIVE_K_SPEED_MAX = 0.22; // 速度映射最大值（不含高角速度加成）
        private static final int DEADZONE_RAMP_FRAMES = 6; // 从进入死区开始到满增益所需帧数
        private static final double DEADZONE_RAMP_MIN = 0.30; // 初始最小比例，避免完全无阻尼
        private static final double HIGH_RATE_THRESHOLD = 0.35; // rad/s 角速度较大判定
        private static final double HIGH_RATE_MULTIPLIER = 1.25; // 高角速度时增益乘子

        // 状态保存用于差分 & 适应
        private double lastHeading = Double.NaN;
        private long lastTimeNs = 0L;
        private double yawRateFiltered = 0.0;
        private int deadzoneFrames = 0; // 连续处于死区的帧计数
        private double currentAdaptiveK = ADAPTIVE_K_MIN; // 记录当前K便于调试

        public TeleOpDriveCommand(
                Drive drive,
                DoubleSupplier x,
                DoubleSupplier y,
                DoubleSupplier rotate,
                DoubleSupplier heading,
                DoubleSupplier speedCoefficient,
                BooleanSupplier useEncoders
        ) {
            this.drive = drive;
            this.x = x;
            this.rotate = rotate;
            this.y = y;
            this.speedCoefficient = speedCoefficient;
            this.heading = heading;
            this.useEncoders = useEncoders;
        }

        @Override
        public void execute() {
            double rotateInput = rotate.getAsDouble();
            double currentHeading = heading.getAsDouble();
            long now = System.nanoTime();

            // 计算 dt（秒）与瞬时角速度
            double dt;
            double yawRateInstant = 0.0;
            if (Double.isNaN(lastHeading)) {
                dt = 0.0; // 首次无差分
                lastHeading = currentHeading;
                lastTimeNs = now;
            } else {
                dt = (now - lastTimeNs) * 1e-9;
                // 防止异常 dt 导致尖峰
                if (dt <= 0 || dt > 0.2) { // 超过200ms认为数据不可靠，重置
                    dt = 0.0;
                    yawRateInstant = 0.0;
                } else {
                    double diff = angleWrap(currentHeading - lastHeading);
                    yawRateInstant = diff / dt; // rad/s
                }
                lastHeading = currentHeading;
                lastTimeNs = now;
            }

            // 一阶低通滤波
            yawRateFiltered = YAW_RATE_ALPHA * yawRateInstant + (1 - YAW_RATE_ALPHA) * yawRateFiltered;

            // 结合 driver 输入与阻尼
            double rotateCmd;
            boolean inDeadzone = Math.abs(rotateInput) < ROTATE_DEADZONE;
            if (inDeadzone) {
                deadzoneFrames++;
                // 计算平移速度幅值（输入向量大小）用于增益放大
                double vx = x.getAsDouble();
                double vy = y.getAsDouble();
                double speedMag = Math.min(1.0, Math.hypot(vx, vy)); // 假设输入归一化到[-1,1]

                // 基础K随速度线性插值
                double baseK = ADAPTIVE_K_MIN + (ADAPTIVE_K_SPEED_MAX - ADAPTIVE_K_MIN) * speedMag;

                // 死区渐进系数
                double ramp = Math.min(1.0, deadzoneFrames / (double) DEADZONE_RAMP_FRAMES);
                ramp = Math.max(ramp, DEADZONE_RAMP_MIN); // 保底

                double adaptiveK = baseK * ramp;

                // 高角速度加成（在大被动旋转时加强阻尼）
                if (Math.abs(yawRateFiltered) > HIGH_RATE_THRESHOLD) {
                    adaptiveK *= HIGH_RATE_MULTIPLIER;
                }
                // 全局限幅
                adaptiveK = Math.min(adaptiveK, ADAPTIVE_K_MAX);
                currentAdaptiveK = adaptiveK; // 记录供调试

                if (Math.abs(yawRateFiltered) > YAW_RATE_NOISE) {
                    // 使用自适应K进行阻尼
                    rotateCmd = rotateInput - adaptiveK * yawRateFiltered;
                } else {
                    rotateCmd = 0.0; // 噪声范围内不输出
                }
            } else {
                // 退出死区，重置计数与保持过滤器衰减
                deadzoneFrames = 0;
                rotateCmd = rotateInput;
                yawRateFiltered *= 0.9;
                currentAdaptiveK = ADAPTIVE_K_MIN; // 回到最小值
            }

            // 限幅
            rotateCmd = clamp(rotateCmd, -MAX_ROTATE_CMD, MAX_ROTATE_CMD);

            double[] potentials = drive.calculateComponents(
                    x.getAsDouble(),
                    y.getAsDouble(),
                    -rotateCmd,
                    currentHeading,
                    speedCoefficient.getAsDouble()
            );
            if (useEncoders.getAsBoolean()) {
                drive.runWithEncoders();
                drive.setVelocity(potentials);
            } else {
                drive.runWithoutEncoders();
                drive.setPower(potentials);
            }
        }

        // 角度包裹到 -pi..pi
        private double angleWrap(double a) {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a < -Math.PI) a += 2 * Math.PI;
            return a;
        }

        private double clamp(double v, double lo, double hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        }

        // 可选：公开当前自适应K（若需要在telemetry中显示，可添加getter）
        public double getCurrentAdaptiveK() { return currentAdaptiveK; }
        public double getYawRateFiltered() { return yawRateFiltered; }
    }
}
