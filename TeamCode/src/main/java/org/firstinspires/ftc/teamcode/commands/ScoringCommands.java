package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import kotlin.NotImplementedError;

public class ScoringCommands {
    public Intake intake;
    public Shooter shooter;
    // public PreShooter preShooter;
    public enum Actions {
        INTAKE_BALL,
        TRANSFER_BALL,
        ACCELERATE_SHOOTER,
        SHOOT_BALL;
    }
    public ScoringCommands(
        Intake intake,
        Shooter shooter
        //PreShooter preShooter
        ) {
        this.intake=intake;
        this.shooter=shooter;
        //this.preShooter=Preshooter;
    }
    public CommandGroupBase scoringAction(Actions action) {
        switch (action){
            case INTAKE_BALL:
                return new SequentialCommandGroup(
                    intake.startIntake()
                    //preShooter.holdBall()
                );
            case TRANSFER_BALL:


        }

    }
}
