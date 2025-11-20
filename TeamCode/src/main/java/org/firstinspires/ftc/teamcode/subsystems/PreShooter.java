package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;

public class PreShooter {
    private DcMotorEx preShooter;
    public PreShooter(@NonNull Hardwares hardwares){
        preShooter=hardwares.motors.preShooter;
    }
    public InstantCommand Hold(){
        return new InstantCommand(
            ()->preShooter.setPower(Constants.preShooterStop)
        );
    }
    public InstantCommand Shoot(){
        return new InstantCommand(
            ()->preShooter.setPower(Constants.preShooterRun)
        );
    }

}
