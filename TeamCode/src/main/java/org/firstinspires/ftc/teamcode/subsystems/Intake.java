package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;

public class Intake {
    private DcMotorEx intakeMotor;
    public Intake(@NonNull Hardwares hardwares){
        intakeMotor = hardwares.motors.intake;
    }

    public InstantCommand startIntake(int useMode){
        if(useMode==1){
            return new InstantCommand(
                    ()->intakeMotor.setPower(Constants.intakePowerClose)
            );
        } else{
            return new InstantCommand(
                    ()->intakeMotor.setPower(0.8)
            );
        }
    }

    public InstantCommand stopIntake(){
        return new InstantCommand(
                ()->intakeMotor.setPower(0)
        );
    }

    public InstantCommand outTake(){
        return new InstantCommand(
                ()->intakeMotor.setPower(Constants.intakePowerOut)
        );
    }
}
