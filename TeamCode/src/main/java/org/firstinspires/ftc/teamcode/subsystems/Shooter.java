package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.jetbrains.annotations.Contract;

public class Shooter {
    private final DcMotorEx shooterFront, shooterBack, preShooter;


    public Shooter(@NonNull Hardwares hardwares) {
        this.shooterFront = hardwares.motors.shooterFront;
        this.shooterBack = hardwares.motors.shooterBack;
        this.preShooter = hardwares.motors.preShooter;
    }

    @NonNull
    @Contract("_ -> new")
    public InstantCommand setShooter(Constants.ShooterConfig config) {
        return new InstantCommand(() -> {
            shooterFront.setVelocity(config.frontVelocity);
            shooterBack.setVelocity(config.backVelocity);
        });
    }

    public InstantCommand allowBallPassClose(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterRunClose)
        );
    }
    public InstantCommand allowBallPassMiddle(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterRunMiddle)
        );
    }
    public InstantCommand allowBallPassFar(){
        return new InstantCommand(
            ()->preShooter.setPower(Constants.preShooterRunFar)
        );
    }

    public InstantCommand blockBallPass(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterBlock)
        );
    }

    public InstantCommand stopPreShooter(){
        return new InstantCommand(
            ()->preShooter.setPower(0)
        );
    }

    public InstantCommand stopShooter() {
        return new InstantCommand(
            ()->{
                shooterFront.setPower(0);
                shooterBack.setPower(0);
            }
        );
    }

    public InstantCommand setShooterFarPIDF(double p, double i, double d, double f) {
        return new InstantCommand(()-> {
            shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        });
    }

    public InstantCommand setShooterClosePIDF() {
        return new InstantCommand(()-> {
            shooterFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        });
    }
}
