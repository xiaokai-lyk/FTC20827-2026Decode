package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.Hardwares;

public class Gate {
    private ServoEx gate;

    public Gate(@NonNull Hardwares hardwares){
        this.gate = hardwares.servos.gate;
    }

    public InstantCommand open(){
        return new InstantCommand(
                () -> gate.turnToAngle(0)
        );
    }

    public InstantCommand close(){
        return new InstantCommand(
                () -> gate.turnToAngle(90)
        );
    }
}
