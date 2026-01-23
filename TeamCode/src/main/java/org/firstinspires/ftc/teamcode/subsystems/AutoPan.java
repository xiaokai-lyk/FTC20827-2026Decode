package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

/**
 * AutoPan Refactored
 * 自动云台控制系统
 *
 * 功能：
 * 1. HOLD 模式：锁定朝前 (0度)
 * 2. TRACK 模式：自动跟踪场地坐标 (0,0)
 * 3. 低通滤波平滑运动，防止抖动
 * 4. 优化的 CAN 总线通信
 */
public class AutoPan {

    // ==========================================
    // 常量配置
    // ==========================================
    public static final double MOTOR_TICKS_PER_REV = 145.6; // 一圈 28 tick 28 × 5.2
    public static final double PAN_REV_PER_MOTOR_REVS = 100.0 / 20.0;
    public static final double PAN_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * PAN_REV_PER_MOTOR_REVS) / 360.0;

    public static final double PAN_MAX_POWER = 0.85;        // 运动最大功率
    public static final double MIN_DISTANCE_CM = 5.0;       // 距离原点太近时不跟踪
    public static final boolean USE_SOFT_LIMIT = true;      // 启用软限位
    public static final double MAX_ANGLE_DEG = 175.0;       // 软限位角度 (留有 5 度余量)

    // 滤波系数 (0.0 - 1.0)，越小越平滑但延迟越高
    // 0.3 是一个经验值，既能滤除高频噪声，又不会造成明显滞后
    private static final double EMA_ALPHA = 0.3;
    private static final double PAN_P_POS = 15, PAN_P_VEL = 30, PAN_I = 0.01, PAN_F = 0, PAN_D = 0;

    // ==========================================
    // 状态定义
    // ==========================================
    public enum Mode {
        HOLD,   // 锁定模式：强制回中 (0度)
        TRACK   // 跟踪模式：指向 (0,0)
    }

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx panMotor;
    private final GoBildaPinpointDriver odo;
    private Mode currentMode = Mode.HOLD;

    // 目标坐标 (场地坐标系)
    private double targetX = 0.0;
    private double targetY = 0.0;

    // 遥测状态
    private boolean isLimitReached = false;
    private double currentRawTarget = 0.0;

    // 滤波相关
    private double currentFilteredAngle = 0.0;

    // 缓存上一次的目标 Ticks，避免重复写入
    private int lastTargetTicks = Integer.MIN_VALUE;

    /**
     * @param hardwares 硬件映射
     * @param targetX 初始目标 X (cm)
     * @param targetY 初始目标 Y (cm)
     */
    public AutoPan(@NonNull Hardwares hardwares, double targetX, double targetY) {
        this.panMotor = hardwares.motors.pan;
        this.odo = hardwares.sensors.odo;
        this.targetX = targetX;
        this.targetY = targetY;
        this.init();
    }

    /**
     * 初始化硬件设置。
     * 注意：此方法会重置编码器零点，请确保上电时云台处于正前方。
     * 此方法不会给电机通电。
     */
    public void init() {
        // 重置编码器，当前位置设为 0
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 设置刹车模式 (断电时抱死)
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panMotor.setPositionPIDFCoefficients(PAN_P_POS);
        panMotor.setVelocityPIDFCoefficients(PAN_P_VEL, PAN_I, PAN_D, PAN_F);

        // 确保初始化时没有功率输出
        panMotor.setPower(0);

        // 重置内部状态
        currentFilteredAngle = 0.0;
        lastTargetTicks = 0;

        // 校准里程计传感器
        odo.recalibrateIMU();
    }

    /**
     * 启动云台。
     * 将模式设为 RUN_TO_POSITION 并锁定在 0 度。
     */
    public void setup() {
        panMotor.setTargetPosition(0);
        panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        panMotor.setPower(PAN_MAX_POWER);

        // 初始状态默认为 HOLD
        this.currentMode = Mode.HOLD;
    }

    /**
     * 切换模式 (在 HOLD 和 TRACK 之间轮转)
     */
    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    public void setMode() {
        if (this.currentMode == Mode.HOLD) {
            this.currentMode = Mode.TRACK;
        } else {
            this.currentMode = Mode.HOLD;
        }
    }

    public Mode getMode() {
        return currentMode;
    }

    /**
     * 遥测数据结构
     */
    public static class TelemetryState {
        public final Mode mode;
        public final double targetX;
        public final double targetY;
        public final double rawTargetAngle;
        public final double filteredAngle;
        public final double currentAngle;
        public final boolean isLimitReached;
        public final double motorPower;

        public TelemetryState(Mode mode, double targetX, double targetY, double rawTargetAngle, double filteredAngle, double currentAngle, boolean isLimitReached, double motorPower) {
            this.mode = mode;
            this.targetX = targetX;
            this.targetY = targetY;
            this.rawTargetAngle = rawTargetAngle;
            this.filteredAngle = filteredAngle;
            this.currentAngle = currentAngle;
            this.isLimitReached = isLimitReached;
            this.motorPower = motorPower;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(java.util.Locale.US, "Mode: %s\nTarget(World): (%.1f, %.1f)\nAngle(Raw/Filt/Curr): %.1f / %.1f / %.1f\nLimit Reached: %b\nPower: %.2f",
                    mode, targetX, targetY, rawTargetAngle, filteredAngle, currentAngle, isLimitReached, motorPower);
        }
    }

    /**
     * 获取遥测状态对象
     */
    public TelemetryState getTelemetryStatus() {
        double currentPosDeg = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
        return new TelemetryState(
                currentMode,
                targetX,
                targetY,
                currentRawTarget,
                currentFilteredAngle,
                currentPosDeg,
                isLimitReached,
                panMotor.getPower()
        );
    }

    /**
     * 在主循环中调用此方法以更新云台状态
     * @param pinpointDriverData 最新的里程计数据
     */
    public void run(PinpointDriverData pinpointDriverData) {
        if (pinpointDriverData == null) return;

        double targetAngleRaw;
        boolean limitActive = false;

        // -----------------------------
        // A. 计算原始目标角度
        // -----------------------------
        if (currentMode == Mode.HOLD) {
            // HOLD 模式：目标永远是 0 度
            targetAngleRaw = 0.0;
        } else {
            // TRACK 模式：计算指向 (targetX, targetY) 的角度
            double rX = pinpointDriverData.getRobotX();
            double rY = pinpointDriverData.getRobotY();
            double heading = pinpointDriverData.getHeadingDegrees();

            // 计算位移向量 (目标 - 机器人)
            double dx = targetX - rX;
            double dy = targetY - rY;

            // 如果距离太近，可能导致角度计算剧变，保持 0 度或上一个角度
            if (Math.hypot(dx, dy) < MIN_DISTANCE_CM) {
                targetAngleRaw = 0.0; // 或者保持 currentFilteredAngle
            } else {
                // atan2 返回世界坐标系角度，减去机器人朝向得到相对云台角度
                double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
                double relativeAngle = normalizeAngle(angleToGoal - heading);

                // 智能限位逻辑：
                // 如果目标在背面 (例如 179 -> -179)，这就构成了穿越 ±180 边界的最短路径。
                // 物理上如果不允许穿越 (墙)，我们希望停留在边界，而不是绕远路 (350度)。

                // 1. 获取当前实际角度
                double currentDeg = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;

                // 2. 计算从当前角度到目标角度的最短“几何”增量 (可能跨越 180 边界)
                double delta = normalizeAngle(relativeAngle - currentDeg);

                // 3. 计算如果不考虑物理墙，我们会去哪里
                double idealNextAngle = currentDeg + delta;

                // 4. 应用软限位 clamp
                // 如果 idealNextAngle 超过了 175，说明要穿墙，被截断在 175。
                // 如果 idealNextAngle 小于 -175，被截断在 -175。
                // 这样就实现了“停在限位处”，而不是尝试用 RUN_TO_POSITION 绕一圈。
                if (USE_SOFT_LIMIT) {
                   if (idealNextAngle > MAX_ANGLE_DEG) {
                       targetAngleRaw = MAX_ANGLE_DEG;
                       limitActive = true;
                   } else if (idealNextAngle < -MAX_ANGLE_DEG) {
                       targetAngleRaw = -MAX_ANGLE_DEG;
                       limitActive = true;
                   } else {
                       targetAngleRaw = idealNextAngle;
                   }
                } else {
                    targetAngleRaw = relativeAngle;
                }
            }
        }

        // 更新遥测变量
        this.currentRawTarget = targetAngleRaw;
        this.isLimitReached = limitActive;

        // -----------------------------
        // B. 滤波算法 (Low Pass Filter) - 6. 防止抖动
        // -----------------------------
        // 使用指数移动平均 (EMA) 算法平滑角度变化
        // New = Alpha * Raw + (1 - Alpha) * Old
        currentFilteredAngle = (EMA_ALPHA * targetAngleRaw) + ((1.0 - EMA_ALPHA) * currentFilteredAngle);

        // -----------------------------
        // C. 硬件执行与优化
        // -----------------------------
        int targetTicks = (int) Math.round(currentFilteredAngle * PAN_TICKS_PER_DEGREE);

        // 7. 优化：仅在目标值发生变化时发送指令，节省 CAN 带宽
        if (targetTicks != lastTargetTicks) {

            panMotor.setTargetPosition(targetTicks);
            // 确保电机处于运行模式 (双重保险)
            if (panMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                panMotor.setPower(PAN_MAX_POWER);
            }

            lastTargetTicks = targetTicks;
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public void setPanMotorPIDF(double pPos, double pVel, double i, double d, double f) {
        panMotor.setPositionPIDFCoefficients(pPos);
        panMotor.setVelocityPIDFCoefficients(pVel, i, d, f);
    }
}

