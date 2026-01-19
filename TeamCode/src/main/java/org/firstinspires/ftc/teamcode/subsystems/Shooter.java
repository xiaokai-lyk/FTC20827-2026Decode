package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardwares;

import java.util.function.DoubleSupplier;

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

    public InstantCommand setShooter(DoubleSupplier velocitySupplier) {
        double velocity = velocitySupplier.getAsDouble();
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
