package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
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
        RESET;
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
    public CommandGroupBase ScoringAction(Actions action,Constants.ShooterConfig shooterConfig) {
        switch (action){
            case INTAKE_BALL:
                return new SequentialCommandGroup(
                    intake.startIntake(),
                    shooter.stopPreShooter()
                );
//            case TRANSFER_BALL:
//                return new SequentialCommandGroup(
//                    intake.startIntake(),
//                    shooter.stopPreShooter()
//                );
//            暂时不需要分开，底盘移动在TeleOpDriveCommand里
            case ACCELERATE_SHOOTER:
                Constants.ShooterConfig finalConfig = shooterConfig != null ?
                    shooterConfig : new Constants.ShooterConfig(1000, 1000);//为了防止没有传shooterConfig但是使用scoringAction
                return new SequentialCommandGroup(
                    shooter.setShooter(shooterConfig)
                );
            case SHOOT_BALL:
                return new SequentialCommandGroup(
                    shooter.runPreShooter()
                );
            case RESET:
                return new SequentialCommandGroup(
                    intake.stopIntake()
                );
        }
        return null;
    }
}
