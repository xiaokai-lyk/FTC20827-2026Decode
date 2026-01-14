package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

public class Intake {
    private static final double INTAKE_POWER_IN = 1;
    private static final double INTAKE_POWER_OUT = -1;
    private DcMotorEx intakeMotor;

    public Intake(@NonNull Hardwares hardwares){
        intakeMotor = hardwares.motors.intake;
    }

    public InstantCommand startIntake(){
        return new InstantCommand(
                ()->intakeMotor.setPower(INTAKE_POWER_IN)
        );
    }

    public InstantCommand stopIntake(){
        return new InstantCommand(
                ()->intakeMotor.setPower(0)
        );
    }

    public InstantCommand outTake(){
        return new InstantCommand(
                ()->intakeMotor.setPower(INTAKE_POWER_OUT)
        );
    }
}
