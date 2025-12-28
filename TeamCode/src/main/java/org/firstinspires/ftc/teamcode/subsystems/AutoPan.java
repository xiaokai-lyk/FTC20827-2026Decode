package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Hardwares;
import java.util.function.Supplier;

public class AutoPan {
    public final Limelight3A limelight;
    private final DcMotorEx panMotor;
    public static final double panTicksPerDegree = 72.1;
    public static final double panMinAngleDegree = -180.0;
    public static final double panMaxAngleDegree = 180.0;
    public static final double panMaxPower = 0.85;

    // 锁定容差（度）
    public static final double PAN_LOCK_TOLERANCE_DEG = 5.0;

    // 视觉控制增益
    public static final double PAN_TX_KP = 0.5;

    // === 绑定状态：由 Subsystem 自主管理 ===
    private boolean bound = false;

    public AutoPan(@NonNull Hardwares hardwares) {
        this.limelight = hardwares.sensors.limelight;
        this.panMotor = hardwares.motors.pan;
    }

    // ==============================
    // Subsystem: 绑定状态管理
    // ==============================

    public boolean isBound() {
        return bound;
    }

    /**
     * 判断目标角度是否超出机械极限
     */
    public boolean shouldBind(double targetAngleDeg) {
        return !isWithinLimits(targetAngleDeg);
    }

    /**
     * 判断是否应从绑定状态退出（当前已绑定 + 目标回到范围内）
     */
    public boolean shouldUnbind(double targetAngleDeg) {
        return bound && isWithinLimits(targetAngleDeg);
    }

    /**
     * 锁定到最近的机械极限
     */
    public void bind(double targetAngleDeg) {
        double norm = normalizeAngle(targetAngleDeg);
        if (norm > 0) {
            setTargetAngle(panMaxAngleDegree);
        } else {
            setTargetAngle(panMinAngleDegree);
        }
        bound = true;
    }

    /**
     * 解除锁定，恢复自由跟踪
     */
    public void unbind(double targetAngleDeg) {
        setTargetAngle(targetAngleDeg);
        bound = false;
    }

    // ==============================
    // 电机控制接口
    // ==============================

    public void setTargetAngle(double angleDegrees) {
        double norm = normalizeAngle(angleDegrees);
        norm = Math.max(
                panMinAngleDegree,
                Math.min(panMaxAngleDegree, norm)
        );
        int targetPos = (int) (norm * panTicksPerDegree);
        panMotor.setTargetPosition(targetPos);
        panMotor.setPower(panMaxPower);
    }

    public double getCurrentAngle() {
        return panMotor.getCurrentPosition() / panTicksPerDegree;
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
        return norm >= panMinAngleDegree && norm <= panMaxAngleDegree;
    }

    // ==============================
    // Command: AutoPanCommand
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
        public void initialize() {
            // 初始化时不清除 bound 状态，由 Subsystem 自主决策
        }

        @Override
        public void execute() {
            OdoData odo = odometerDataSupplier;
            if (odo == null) {
                autoPan.hold();
                return;
            }

            // === 1. 计算 Odo 预瞄角度（球门 = (0,0)）===
            double robotX = odo.getRobotVx(); // 相对于球门的 x
            double robotY = odo.getRobotVy(); // 相对于球门的 y
            double dx = -robotX;              // 指向原点的向量
            double dy = -robotY;
            double angleToGoalRad = Math.atan2(dy, dx);
            double robotHeadingRad = odo.getHeadingRadians();
            double odoTargetAngleDeg = Math.toDegrees(angleToGoalRad - robotHeadingRad);

            // === 2. 获取视觉数据 ===
            LLResult result = autoPan.limelight.getLatestResult();
            boolean hasValidVision = (result != null && result.isValid());

            // === 3. 确定当前目标角度 ===
            double currentTargetAngleDeg;
            if (hasValidVision) {
                // 视觉可用：使用视觉微调
                double tx = result.getTx();
                double currentAngle = autoPan.getCurrentAngle();
                currentTargetAngleDeg = currentAngle - tx * PAN_TX_KP;
            } else {
                // 视觉不可用：使用 Odo 预瞄（即使超出范围也先保留值）
                currentTargetAngleDeg = odoTargetAngleDeg;
            }

            // === 4. 委托 Subsystem 决策绑定/跟踪 ===
            if (!autoPan.isBound()) {
                // 当前未绑定
                if (autoPan.shouldBind(currentTargetAngleDeg)) {
                    autoPan.bind(currentTargetAngleDeg);
                } else {
                    autoPan.setTargetAngle(currentTargetAngleDeg);
                }
            } else {
                // 当前已绑定
                if (autoPan.shouldUnbind(currentTargetAngleDeg)) {
                    autoPan.unbind(currentTargetAngleDeg);
                } else {
                    // 维持锁定：保持在极限位置
                    double current = autoPan.getCurrentAngle();
                    if (Math.abs(current - panMaxAngleDegree) < 5) {
                        autoPan.setTargetAngle(panMaxAngleDegree);
                    } else {
                        autoPan.setTargetAngle(panMinAngleDegree);
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
