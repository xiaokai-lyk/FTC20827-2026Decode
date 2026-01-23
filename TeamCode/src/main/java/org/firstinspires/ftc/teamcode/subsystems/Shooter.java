package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardwares;

import java.util.function.DoubleSupplier;

/**
 * 发射系统
 *
 * 功能：
 * 1. 控制发射轮速度
 * 2. 控制俯仰角度
 * 3. 提供预设的射击配置
 * 4. 获取发射系统遥测状态
 */
public class Shooter {

    // ==========================================
    // 常量配置
    // ==========================================
    public static final ShooterConfig shooter40cm = new ShooterConfig(1000, 50);    // 40cm射击配置
    public static final ShooterConfig shooter125cm = new ShooterConfig(1500, 50);    // 125cm射击配置
    public static final ShooterConfig shooter250cm = new ShooterConfig(2300, 50);    // 250cm射击配置

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx shooterLeft, shooterRight;
    private final ServoEx pitch;

    public static class ShooterConfig {
        public double shooterVelocity, pitchAngle;

        /**
         * 构造发射配置
         * @param shooterVelocity 发射轮速度
         * @param pitchAngle 俯仰角度
         */
        public ShooterConfig(double shooterVelocity, double pitchAngle) {
            this.shooterVelocity = shooterVelocity;
            this.pitchAngle = pitchAngle;
        }
    }

    /**
     * 构造发射系统实例
     * @param hardwares 硬件映射
     */
    public Shooter(@NonNull Hardwares hardwares) {
        this.shooterLeft = hardwares.motors.shooterLeft;
        this.shooterRight = hardwares.motors.shooterRight;
        this.pitch = hardwares.servos.pitch;
        this.init();
    }

    private void init() {
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * 获取设置发射轮速度和俯仰角度的命令
     * @param config 发射配置
     * @return 设置发射轮速度和俯仰角度的InstantCommand
     */
    public InstantCommand setShooterAndPitch(ShooterConfig config) {
        return new InstantCommand(
                () -> {
                    shooterLeft.setVelocity(config.shooterVelocity);
                    shooterRight.setVelocity(config.shooterVelocity);
                    pitch.turnToAngle(config.pitchAngle);
                }
        );
    }

    /**
     * 获取设置发射轮速度的命令
     * @param velocitySupplier 速度提供者
     * @return 设置发射轮速度的InstantCommand
     */
    public InstantCommand setShooter(@NonNull DoubleSupplier velocitySupplier) {
        double velocity = velocitySupplier.getAsDouble();
        return new InstantCommand(
                () -> {
                    shooterLeft.setVelocity(velocity);
                    shooterRight.setVelocity(velocity);
                }
        );
    }

    /**
     * 获取发射轮怠速的命令
     * @return 发射轮怠速的InstantCommand
     */
    public InstantCommand shooterIdle() {
        return new InstantCommand(()->{
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterLeft.setPower(0.3);
            shooterRight.setPower(0.3);
        });
    }
    /**
    * 获取发射轮停止的命令
    * @return 发射轮停止的InstantCommand
    */
    public InstantCommand stopShooter() {
        return new InstantCommand(
                () -> {
                    shooterLeft.setMotorDisable();
                    shooterRight.setMotorDisable();
                }
        );
    }

    /**
     * 获取设置俯仰角度的命令
     * @param angleSupplier 俯仰角度提供者
     * @return 设置俯仰角度的InstantCommand
     */
    public InstantCommand setPitch(@NonNull DoubleSupplier angleSupplier) {
        double angle = angleSupplier.getAsDouble();
        return new InstantCommand(() -> pitch.turnToAngle(angle));
    }

    /**
     * 获取设置发射轮PIDF参数的命令
     * @param p P参数
     * @param i I参数
     * @param d D参数
     * @param f F参数
     * @return 设置发射轮PIDF参数的InstantCommand
     */
    public InstantCommand setShooterPIDF(double p, double i, double d, double f) {
        return new InstantCommand(()-> {
            shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        });
    }

    public InstantCommand resetShooterPIDF() {
        return new InstantCommand(()-> {
            shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        });
    }

    public static class TelemetryState {
        public final double leftVelocity;
        public final double rightVelocity;
        public final double pitchAngle;
        public final double leftCurrent;
        public final double rightCurrent;

        public TelemetryState(double leftVelocity,
                              double rightVelocity,
                              double pitchAngle,
                              double leftCurrent,
                              double rightCurrent) {
            this.leftVelocity = leftVelocity;
            this.rightVelocity = rightVelocity;
            this.pitchAngle = pitchAngle;
            this.leftCurrent = leftCurrent;
            this.rightCurrent = rightCurrent;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(java.util.Locale.US, "Left Velocity: %.1f\nRight Velocity: %.1f\nPitch Angle: %.1f\nLeft Current: %.1f\nRight Current: %.1f",
                    leftVelocity, rightVelocity, pitchAngle, leftCurrent, rightCurrent);
        }
    }

    /**
     * 获取发射系统的遥测状态
     * @return 发射系统的遥测状态
     */
    public TelemetryState getTelemetryState() {
        return new TelemetryState(
                shooterLeft.getVelocity(),
                shooterRight.getVelocity(),
                pitch.getAngle(),
                shooterLeft.getCurrent(CurrentUnit.MILLIAMPS),
                shooterRight.getCurrent(CurrentUnit.MILLIAMPS)
        );
    }
}
