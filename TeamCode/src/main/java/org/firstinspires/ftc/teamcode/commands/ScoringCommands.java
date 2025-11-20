package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
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
        SHOOT_BALL,
        RESET
    }
    public ScoringCommands(Intake intake, Shooter shooter) {
        this.intake=intake;
        this.shooter=shooter;
    }

    public CommandGroupBase scoringAction(Actions action) {
        return this.scoringAction(action, new Constants.ShooterConfig(1000, 1000));//没有就用默认
    }

    public CommandGroupBase scoringAction(Actions action,Constants.ShooterConfig shooterConfig) {
        switch (action){
            case INTAKE_BALL:
                return new ParallelCommandGroup(
                    intake.startIntake(),
                    shooter.stopPreShooter()
                );
            case ACCELERATE_SHOOTER:
                return new ParallelCommandGroup(
                    shooter.setShooter(shooterConfig)
                );
            case SHOOT_BALL:
                return new ParallelCommandGroup(
                    shooter.runPreShooter()
                );
            case RESET:
                return new ParallelCommandGroup(
                    intake.stopIntake()
                );
        }
        return null;
    }
}
