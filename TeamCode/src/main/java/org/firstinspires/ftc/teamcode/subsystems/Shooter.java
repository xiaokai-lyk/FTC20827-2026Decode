package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.XKPIDFController;

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
    public static final ShooterConfig shooter40cm = new ShooterConfig(1300, 0);
    public static final ShooterConfig shooterNearTop = new ShooterConfig(1500, 120);
    public static final ShooterConfig shooterFar = new ShooterConfig(1850, 290);
    public static final ShooterConfig shooterUltraFar = new ShooterConfig(1925, 290);

    // Default PIDF constants (tuned values)
    public static double kP = 0.0003;
    public static double kI = 0.0001;
    public static double kD = 0.0;
    public static double kF = 0.00037;

    // Bang-Bang params
    public static boolean ENABLE_BANG_BANG = true;
    public static double BANG_BAND_ABS = 30.0;
    public static double ASSIST_POWER = 1.0;

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx shooterLeft, shooterRight;
    private final ServoEx pitch;
    private final XKPIDFController controller;
    private boolean isClosedLoop = false;
    private XKPIDFController.Output controllerOutput;

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
        this.controller = new XKPIDFController(hardwares.sensors.voltageSensor);
        this.init();
    }

    private void init() {
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller.setPIDF(kP, kI, kD, kF)
                .setBangBang(ENABLE_BANG_BANG, BANG_BAND_ABS, ASSIST_POWER);
    }


    public void run() {
        if (isClosedLoop) {
            double velLeft = shooterLeft.getVelocity();
            double velRight = shooterRight.getVelocity();
            double avgVel;
            if (Math.max(velLeft, velRight) - Math.min(velLeft, velRight) > 200){
                avgVel = Math.max(velLeft, velRight);
            }else{
                avgVel = (velLeft + velRight) / 2.0;
            }
            this.controllerOutput = controller.update(avgVel);
            shooterLeft.setPower(this.controllerOutput.power);
            shooterRight.setPower(this.controllerOutput.power);
        }
    }

    /**
     * 获取设置发射轮速度和俯仰角度的命令
     * @param config 发射配置
     * @return 设置发射轮速度和俯仰角度的InstantCommand
     */
    public InstantCommand setShooterConfig(ShooterConfig config) {
        return new InstantCommand(
                () -> {
                    isClosedLoop = true;
                    controller.setTargetVelocity(config.shooterVelocity);
                    pitch.turnToAngle(config.pitchAngle);
                }
        );
    }


    /**
     * 获取发射轮怠速的命令
     * @return 发射轮怠速的InstantCommand
     */
    public InstantCommand shooterIdle() {
        return new InstantCommand(()->{
            isClosedLoop = false;
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
                    isClosedLoop = false;
                    controller.reset();
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                }
        );
    }

    /**
     * 获取设置俯仰角度的命令
     * @param angle 俯仰角度
     * @return 设置俯仰角度的InstantCommand
     */
    public InstantCommand setPitch(double angle) {
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
            controller.setPIDF(p, i, d, f);
        });
    }

    public static class TelemetryState {
        public final double leftVelocity;
        public final double rightVelocity;
        public final double pitchAngle;
        public final double leftCurrent;
        public final double rightCurrent;
        public final String controllerOutput;

        public TelemetryState(double leftVelocity,
                              double rightVelocity,
                              double pitchAngle,
                              double leftCurrent,
                              double rightCurrent,
                              String controllerOutput) {
            this.leftVelocity = leftVelocity;
            this.rightVelocity = rightVelocity;
            this.pitchAngle = pitchAngle;
            this.leftCurrent = leftCurrent;
            this.rightCurrent = rightCurrent;
            this.controllerOutput = controllerOutput;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(java.util.Locale.US, "Left Velocity: %.1f\nRight Velocity: %.1f\nPitch Angle: %.1f\nLeft Current: %.1f\nRight Current: %.1f\nController Output: %s",
                    leftVelocity, rightVelocity, pitchAngle, leftCurrent, rightCurrent, controllerOutput);
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
                shooterRight.getCurrent(CurrentUnit.MILLIAMPS),
                controllerOutput != null ? controllerOutput.toString() : "N/A"
        );
    }
}
