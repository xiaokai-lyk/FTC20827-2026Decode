package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.Hardwares;

/**
 * AutoPan: 基于 GoBILDA Pinpoint 的有限旋转自动云台（±180°）
 *
 * 特性：
 * - 无视觉依赖，纯 Odo 驱动
 * - 云台物理范围限制为 [-180°, +180°]（因理线限制）
 * - 所有目标角度自动归一化并限制在此范围内
 * - 球门坐标系：红方球门位于 (0, 0)
 */
public class Pan {
    private final DcMotorEx panMotor;

    // ==============================
    // 🔧 硬件配置（根据你的实际结构修改）
    // ==============================

    // GoBILDA 5203 系列 Yellow Jacket 3.7:1 电机（内部减速）
    public static final double MOTOR_TICKS_PER_REV = 103.6; // 一圈 28 tick 28 × 3.7

    // 外部齿轮比：电机输出轴齿数 / 云台轴齿数 = 100 / 20
    public static final double PAN_REV_PER_MOTOR_REVS = 100.0 / 20.0;

    // 云台每度对应的电机 ticks
    public static final double PAN_TICKS_PER_DEGREE =
            (MOTOR_TICKS_PER_REV * PAN_REV_PER_MOTOR_REVS) / 360.0;

    // 电机最大功率
    public static final double PAN_MAX_POWER = 0.85;

    // 最小有效跟踪距离（厘米），避免在原点附近抖动
    public static final double MIN_DISTANCE_CM = 5.0;

    // （可选）如果你的云台实际不能完全到 ±180°，可启用软限位
    // 例如：限制到 ±170° 防止拉线过紧
    public static final boolean USE_SOFT_LIMIT = true;
    public static final double MAX_ANGLE_DEG = 170.0; // 仅当 USE_SOFT_LIMIT = true 时生效

    // ==============================
    // 构造函数
    // ==============================

    public Pan(@NonNull Hardwares hardwares) {
        this.panMotor = hardwares.motors.pan;

        // 初始化编码器
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // ==============================
    // 核心控制方法
    // ==============================

    /**
     * 设置云台目标角度（单位：度）
     * 自动归一化到 [-180, 180)，并可选软限幅。
     */
    public void setTargetAngle(double panAngleDegrees) {
        // 归一化到 [-180, 180)
        double normalized = normalizeAngle(panAngleDegrees);

        if (USE_SOFT_LIMIT) {
            normalized = Math.max(-MAX_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, normalized));
        }

        int targetTicks = (int) Math.round(normalized * PAN_TICKS_PER_DEGREE);

        // 优化：仅在目标位置改变时发送指令，减少 CAN 总线负载
        if (panMotor.getTargetPosition() != targetTicks) {
            panMotor.setTargetPosition(targetTicks);
            panMotor.setPower(PAN_MAX_POWER);
        }
    }

    /**
     * 停止云台（刹车）
     */
    public void hold() {
        panMotor.setPower(0.0);
    }

    // ==============================
    // 工具方法
    // ==============================

    /**
     * 将任意角度归一化到 [-180, 180)
     */
    public static double normalizeAngle(double angleDeg) {
        while (angleDeg >= 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    public double getCurrentPanDeg(){
        return panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
    }

    // ==============================
    // Command: 实时跟踪红方球门 (0, 0)
    // ==============================

    public static class AutoPanCommand extends CommandBase {
        private Supplier<OdoData> odoDataSupplier;
        private final Pan pan;

        /**
         * @param pan   云台子系统
         * @param odoDataSupplier  获取最新 OdoData
         */
        public AutoPanCommand(Pan pan, Supplier<OdoData> odoDataSupplier) {
            this.pan = pan;
            this.odoDataSupplier = odoDataSupplier;
        }

        @Override
        public void execute() {
            OdoData odoData = odoDataSupplier.get();

            // 实时获取最新位置和航向
            double x = odoData.getRobotX();      // 机器人 X 坐标（cm）
            double y = odoData.getRobotY();      // 机器人 Y 坐标（cm）
            double headingDeg = odoData.getHeadingDegrees();

            // 计算到球门 (0,0) 的向量
            double dx = -x;
            double dy = -y;
            double distance = Math.hypot(dx, dy);

            if (distance < MIN_DISTANCE_CM) {
                pan.hold();
                return;
            }

            // 计算指向球门的世界坐标系角度
            double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));

            // 转换为云台相对于机器人机身的目标角度（度）
            double targetAngleDeg =(angleToGoalDeg - headingDeg);

            // 自动归一化并限制在 [-180, 180]
            pan.setTargetAngle(targetAngleDeg);
        }

        @Override
        public void end(boolean interrupted) {
            pan.hold();
        }

        @Override
        public boolean isFinished() {
            return false; // 持续运行
        }
    }
}