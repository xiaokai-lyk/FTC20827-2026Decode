package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.jetbrains.annotations.Contract;

public class Shooter {
    private final DcMotorEx shooterFront, shooterBack, preShooter, intake;
    private double shooterFrontTarget = 0;
    private double shooterBackTarget = 0;

    public Shooter(@NonNull Hardwares hardwares) {
        this.shooterFront = hardwares.motors.shooterFront;
        this.shooterBack = hardwares.motors.shooterBack;
        this.preShooter = hardwares.motors.preShooter;
        this.intake = hardwares.motors.intake;
    }

    @NonNull
    @Contract("_ -> new")
    public InstantCommand setShooter(Constants.ShooterConfig config) {
        return new InstantCommand(() -> {
            shooterFront.setVelocity(config.frontVelocity);
            shooterBack.setVelocity(config.backVelocity);
            shooterFrontTarget = config.frontVelocity;
            shooterBackTarget = config.backVelocity;
        });
    }

    public InstantCommand runPreShooter(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterRun)
        );
    }

    public InstantCommand stopPreShooter(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterStop)
        );
    }

    public boolean shooterReady(double threshold) {
        double frontVelocity = shooterFront.getVelocity();
        double backVelocity = shooterBack.getVelocity();
        return Math.abs(frontVelocity - shooterFrontTarget) < threshold &&
               Math.abs(backVelocity - shooterBackTarget) < threshold;
    }

    public boolean shooterReady(){
        return this.shooterReady(50);
    }
}