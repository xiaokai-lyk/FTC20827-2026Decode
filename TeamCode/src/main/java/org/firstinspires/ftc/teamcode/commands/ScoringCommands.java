package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import kotlin.NotImplementedError;

public class ScoringCommands {
    public enum Actions {
        INTAKE_BALL,
        TRANSFER_BALL,
        ACCELERATE_SHOOTER,
        SHOOT_BALL
    }

    public CommandGroupBase scoringAction(Actions action) {
        throw new NotImplementedError();
    }
}
