package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

import java.util.function.Supplier;

public class AutoPan {
    public final Limelight3A limelight;
    private final DcMotorEx panMotor;

    // ==============================
    // 🔧 配置常量（请根据实测填写！）
    // ==============================

    // 1. 电机自身：每转多少 ticks（实测或查型号）
    public static final double MOTOR_TICKS_PER_REV = 384.0; // ← 替换为你测得的电机 ticks/rev

    // 2. 传动比：电机转多少圈，云台才转 1 圈
    //    例如：电机:云台 = 3:1 ⇒ motorRevsPerPanRev = 3.0
    public static final double MOTOR_REVS_PER_PAN_REV = 3.0; // ← 替换为你的齿轮比！

    // 3. 推导出：云台每度对应多少电机 ticks
    public static final double PAN_TICKS_PER_DEGREE =
            (MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_PAN_REV) / 360.0;

    // 4. 云台物理极限（以云台实际角度为准）
    public static final double PAN_MIN_ANGLE_DEG = -180.0;
    public static final double PAN_MAX_ANGLE_DEG = 180.0;

    // 5. 其他参数
    public static final double PAN_MAX_POWER = 0.85;
    public static final double PAN_LOCK_TOLERANCE_DEG = 5.0;
    public static final double PAN_TX_KP = 0.5;

    // === 绑定状态 ===
    private boolean bound = false;

    public AutoPan(@NonNull Hardwares hardwares) {
        this.limelight = hardwares.sensors.limelight;
        this.panMotor = hardwares.motors.pan;

        // 初始化电机模式
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // ==============================
    // Subsystem: 绑定状态管理
    // ==============================

    public boolean isBound() {
        return bound;
    }

    public boolean shouldBind(double targetAngleDeg) {
        return !isWithinLimits(targetAngleDeg);
    }

    public boolean shouldUnbind(double targetAngleDeg) {
        return bound && isWithinLimits(targetAngleDeg);
    }

    public void bind(double targetAngleDeg) {
        double norm = normalizeAngle(targetAngleDeg);
        if (norm > 0) {
            setTargetAngle(PAN_MAX_ANGLE_DEG);
        } else {
            setTargetAngle(PAN_MIN_ANGLE_DEG);
        }
        bound = true;
    }

    public void unbind(double targetAngleDeg) {
        setTargetAngle(targetAngleDeg);
        bound = false;
    }

    // ==============================
    // 电机控制接口（核心：使用 PAN_TICKS_PER_DEGREE 换算）
    // ==============================

    public void setTargetAngle(double panAngleDegrees) {
        double norm = normalizeAngle(panAngleDegrees);
        norm = Math.max(PAN_MIN_ANGLE_DEG, Math.min(PAN_MAX_ANGLE_DEG, norm));
        int targetTicks = (int) (norm * PAN_TICKS_PER_DEGREE);
        panMotor.setTargetPosition(targetTicks);
        panMotor.setPower(PAN_MAX_POWER);
    }

    public double getCurrentAngle() {
        return panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
    }

    public void hold() {
        panMotor.setPower(0);
    }

    // ==============================
    // 工具方法
    // ==============================

    private double normalizeAngle(double angleDeg) {
        while (angleDeg >= 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }

    public boolean isWithinLimits(double angleDeg) {
        double norm = normalizeAngle(angleDeg);
        return norm >= PAN_MIN_ANGLE_DEG && norm <= PAN_MAX_ANGLE_DEG;
    }

    // ==============================
    // Command
    // ==============================

    public static class AutoPanCommand extends CommandBase {
        private final OdoData odometerDataSupplier;
        private final AutoPan autoPan;

        public AutoPanCommand(
                AutoPan autoPan,
                OdoData odometerDataSupplier) {
            this.autoPan = autoPan;
            this.odometerDataSupplier = odometerDataSupplier;
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            OdoData odo = odometerDataSupplier;
            if (odo == null) {
                autoPan.hold();
                return;
            }

            // 计算 Odo 预瞄角度（球门 = (0,0)）
            double robotX = odo.getRobotVx();
            double robotY = odo.getRobotVy();
            double dx = -robotX;
            double dy = -robotY;
            double angleToGoalRad = Math.atan2(dy, dx);
            double robotHeadingRad = odo.getHeadingRadians();
            double odoTargetAngleDeg = Math.toDegrees(angleToGoalRad - robotHeadingRad);

            // 获取视觉
            LLResult result = autoPan.limelight.getLatestResult();
            boolean hasValidVision = (result != null && result.isValid());

            double currentTargetAngleDeg;
            if (hasValidVision) {
                double tx = result.getTx();
                double currentAngle = autoPan.getCurrentAngle();
                currentTargetAngleDeg = currentAngle - tx * PAN_TX_KP;
            } else {
                currentTargetAngleDeg = odoTargetAngleDeg;
            }

            // 状态决策
            if (!autoPan.isBound()) {
                if (autoPan.shouldBind(currentTargetAngleDeg)) {
                    autoPan.bind(currentTargetAngleDeg);
                } else {
                    autoPan.setTargetAngle(currentTargetAngleDeg);
                }
            } else {
                if (autoPan.shouldUnbind(currentTargetAngleDeg)) {
                    autoPan.unbind(currentTargetAngleDeg);
                } else {
                    double current = autoPan.getCurrentAngle();
                    if (Math.abs(current - PAN_MAX_ANGLE_DEG) < PAN_LOCK_TOLERANCE_DEG) {
                        autoPan.setTargetAngle(PAN_MAX_ANGLE_DEG);
                    } else {
                        autoPan.setTargetAngle(PAN_MIN_ANGLE_DEG);
                    }
                }
            }
        }

        @Override
        public void end(boolean interrupted) {
            autoPan.hold();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}