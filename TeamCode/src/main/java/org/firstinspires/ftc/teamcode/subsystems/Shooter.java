package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardwares;

public class Shooter {
    public static ShooterConfig shooter40cm = new ShooterConfig(1100, 50);
    public static ShooterConfig shooter125cm = new ShooterConfig(500, 50);
    public static ShooterConfig shooter250cm = new ShooterConfig(860, 50);
    public static ShooterConfig shooterFar = new ShooterConfig(330, 50);
    private final DcMotorEx shooterLeft, shooterRight;
    private final ServoEx pitch;

    public static class ShooterConfig {
        public double shooterVelocity, pitchAngle;
        public ShooterConfig(double shooterVelocity, double pitchAngle) {
            this.shooterVelocity = shooterVelocity;
            this.pitchAngle = pitchAngle;
        }
    }

    public Shooter(@NonNull Hardwares hardwares) {
        this.shooterLeft = hardwares.motors.shooterLeft;
        this.shooterRight = hardwares.motors.shooterRight;
        this.pitch = hardwares.servos.pitch;
    }

    public InstantCommand setShooterAndPitch(ShooterConfig config) {
        return new InstantCommand(
                () -> {
                    shooterLeft.setVelocity(config.shooterVelocity);
                    shooterRight.setVelocity(config.shooterVelocity);
                    pitch.turnToAngle(config.pitchAngle);
                }
        );
    }

    public InstantCommand setShooter(double velocity) {
        return new InstantCommand(
                () -> {
                    shooterLeft.setVelocity(velocity);
                    shooterRight.setVelocity(velocity);
                }
        );
    }

    public InstantCommand stopShooter() {
        return new InstantCommand(
                () -> {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                }
        );
    }

    public InstantCommand setPitch(double angle) {
        return new InstantCommand(() -> pitch.turnToAngle(angle));
    }

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

    public double getShooterLeftVelocity() {
        return shooterLeft.getVelocity();
    }

    public double getShooterRightVelocity() {
        return shooterRight.getVelocity();
    }

    public double getPitchAngle() {
        return pitch.getAngle();
    }
}
