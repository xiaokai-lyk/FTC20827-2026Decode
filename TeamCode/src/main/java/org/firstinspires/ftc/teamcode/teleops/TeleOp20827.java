package org.firstinspires.ftc.teamcode.teleops;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@TeleOp(name = "TeleOp", group = "teleops")
public class TeleOp20827 extends XKCommandOpmode {
    private Shooter shooter;
    private Hardwares hardwares;
    private Intake intake;
    private GamepadEx gamepad1, gamepad2;
    private Constants.ShooterConfig shooterConfig;
    protected Drive.DriveCommand driveCommand;
    @Override
    public void initialize() {
        this.gamepad1 = new GamepadEx(super.gamepad1);
        this.gamepad2 = new GamepadEx(super.gamepad2);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);


        Drive drive = new Drive(hardwares);


        driveCommand = new Drive.DriveCommand(
            drive,
            gamepad1::getLeftX,
            gamepad1::getLeftY,
            ()->-gamepad1.getRightX(),
            ()-> new OdometerData(hardwares.sensors.odo),
            ()-> (gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 ) ? 0.5 : 1,
            ()-> true,
            ()->true
        );

        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void onStart() {
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        hardwares.sensors.odo.update();
        telemetry.addData("Heading (Rad)", hardwares.sensors.odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Heading (Deg)", hardwares.sensors.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X", hardwares.sensors.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Y", hardwares.sensors.odo.getPosY(DistanceUnit.MM));
        telemetry.addLine("---");

        telemetry.addData("drive mode", hardwares.motors.mFrontLeft.getMode().toString());
        telemetry.addData("front left velocity", hardwares.motors.mFrontLeft.getVelocity());
        telemetry.addData("front right velocity", hardwares.motors.mFrontRight.getVelocity());
        telemetry.addData("back left velocity", hardwares.motors.mBackLeft.getVelocity());
        telemetry.addData("back right velocity", hardwares.motors.mBackRight.getVelocity());

        telemetry.addLine("---");
        telemetry.addData("X damping", driveCommand.dampedX - gamepad1.getLeftX());
        telemetry.addData("Y damping", driveCommand.dampedY - gamepad1.getLeftY());
        telemetry.addData("Rotate damping", driveCommand.dampedRotate - (-gamepad1.getRightX()));

        telemetry.update();
    }


    @Override
    public void functionalButtons() {
        ParallelCommandGroup enterRunningMode = new ParallelCommandGroup(
                shooter.stopPreShooter(),
                intake.stopIntake()
        );

        // ConditionalCommand that runs enterRunningMode only when neither trigger is pressed.
        ConditionalCommand stopIfNoTriggers = new ConditionalCommand(
                enterRunningMode,
                new InstantCommand(() -> {}),
                () -> {
                    boolean rt = gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
                    boolean lt = gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
                    return (!rt && !lt && !gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER));
                }
        );

        new ButtonEx(
                () -> gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        ).whenPressed(
            shooter.blockBallPass(),
            intake.startIntake(true)
        ).whenReleased(
            stopIfNoTriggers
        );

        new ButtonEx(
                ()-> gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        ).whenPressed(
                shooter.allowBallPass(),
                intake.startIntake(false)
        ).whenReleased(
            stopIfNoTriggers
        );

        new ButtonEx(
            ()-> gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(
            intake.outTake()
        ).whenReleased(
            stopIfNoTriggers
        );



        new ButtonEx(
            ()-> gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
            ()->hardwares.sensors.odo.setHeading(0,AngleUnit.DEGREES)
        );


        new ButtonEx(
            ()-> gamepad2.getButton(GamepadKeys.Button.B)
        ).whenPressed(
            /* Assumption: shooter125cm corresponds to HIGH mode */
            shooter.setShooter(Constants.shooter125cm)
        );

        new ButtonEx(
            ()-> gamepad2.getButton(GamepadKeys.Button.Y)
        ).whenPressed(
            /* Assumption: shooter40cm corresponds to LOW mode */
            shooter.setShooter(Constants.shooter40cm)
        );

        new ButtonEx(
            ()-> gamepad2.getButton(GamepadKeys.Button.A)
        ).whenPressed(
            /* Assumption: shooter40cm corresponds to LOW mode */
            shooter.setShooter(Constants.shooterFar)
        );
    }
}
