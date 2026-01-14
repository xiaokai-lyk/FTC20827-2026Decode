package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.legacy.Constants;
import org.jetbrains.annotations.Contract;

public class Shooter {
    public static ShooterConfig shooter40cm = new ShooterConfig(1100, 500);
    public static ShooterConfig shooter125cm = new ShooterConfig(500, 1200);
    public static ShooterConfig shooter250cm = new ShooterConfig(860, 1660);
    public static ShooterConfig shooterFar = new ShooterConfig(330, 2100);
    private final DcMotorEx shooterFront, shooterBack;

    public static class ShooterConfig {
        public int frontVelocity;
        public int backVelocity;
        public ShooterConfig(int frontVelocity, int backVelocity) {
            this.frontVelocity = frontVelocity;
            this.backVelocity = backVelocity;
        }
    }

    public Shooter(@NonNull Hardwares hardwares) {
        this.shooterFront = hardwares.motors.shooterFront;
        this.shooterBack = hardwares.motors.shooterBack;
    }

    @NonNull
    @Contract("_ -> new")
    public InstantCommand setShooter(ShooterConfig config) {
        return new InstantCommand(() -> {
            shooterFront.setVelocity(config.frontVelocity);
            shooterBack.setVelocity(config.backVelocity);
        });
    }

    public InstantCommand stopShooter() {
        return new InstantCommand(
                ()->{
                    shooterFront.setPower(0);
                    shooterBack.setPower(0);
                }
        );
    }

    public InstantCommand setShooterPIDF(double p, double i, double d, double f) {
        return new InstantCommand(()-> {
            shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        });
    }

    public InstantCommand resetShooterPIDF() {
        return new InstantCommand(()-> {
            shooterFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        });
    }

    public boolean shooterReady(Constants.ShooterConfig targetVelocity){
        if (Math.abs(shooterFront.getVelocity()-targetVelocity.frontVelocity) > 100 && Math.abs(shooterBack.getVelocity()-targetVelocity.backVelocity) > 100) {
            return false;
        }
        return true;
    }
}
