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

    public InstantCommand startIntake(boolean useFastMode){
        if(useFastMode){
            return new InstantCommand(
                    ()->intakeMotor.setPower(Constants.intakePowerFast)
            );
        }else{
            return new InstantCommand(
                ()->intakeMotor.setPower(Constants.intakePowerSlow)
            );
        }
    }

    public InstantCommand stopIntake(){
        return new InstantCommand(
                ()->intakeMotor.setPower(0)
        );
    }
}
