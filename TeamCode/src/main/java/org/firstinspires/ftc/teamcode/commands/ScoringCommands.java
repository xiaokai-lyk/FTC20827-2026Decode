package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ScoringCommands {
    public Intake intake;
    public Shooter shooter;
    // public PreShooter preShooter;
    public enum Actions {
        INTAKE_BALL,
        SHUT_DOWN_INTAKE,
        ACCELERATE_SHOOTER,
        SHOOT_BALL
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
                    shooter.blockBallPass()
                );
            case ACCELERATE_SHOOTER:
                return new ParallelCommandGroup(
                    shooter.setShooter(shooterConfig)
                );
            case SHOOT_BALL:
                return new ParallelCommandGroup(
                    shooter.allowBallPass()
                );
            case SHUT_DOWN_INTAKE:
                return new ParallelCommandGroup(
                    shooter.stopPreShooter(),
                    intake.stopIntake()
                );

        }
        return null;
    }
}
