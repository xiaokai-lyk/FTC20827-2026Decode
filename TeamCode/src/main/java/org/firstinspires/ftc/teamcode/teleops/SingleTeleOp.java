package org.firstinspires.ftc.teamcode.teleops;



import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@Disabled
@TeleOp(name = "SingleTeleOp", group = "teleops")
public class SingleTeleOp extends XKCommandOpmode {
    private Shooter shooter;
    private Hardwares hardwares;
    private Intake intake;
    private Light light;
    private GamepadEx gamepad1;
    private Constants.ShooterConfig shooterConfig;
    protected Drive.DriveCommand driveCommand;
    @Override
    public void initialize() {
        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        light = new Light(hardwares);


        Drive drive = new Drive(hardwares);


        driveCommand = new Drive.DriveCommand(
                drive,
                gamepad1::getLeftX,
                gamepad1::getLeftY,
                ()->-gamepad1.getRightX(),
                ()-> new OdometerData(hardwares.sensors.odo),
                ()->1,
                ()-> true,
            ()->true);

        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void onStart() {
        light.setLightColor(255, 255, 255).schedule();
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
        new ButtonEx(
                () -> gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        ).whenPressed(
            shooter.blockBallPass(),
            intake.startIntake(true)
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(
                shooter.allowBallPass(),
                intake.startIntake(false)
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
                shooter.setShooter(Constants.shooterStop)
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.X)
        ).whenPressed(
            shooter.setShooter(Constants.shooter250cm),
            light.setLightColor(255, 0, 0)
        );

        new ButtonEx(
            ()-> gamepad1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenPressed(
            ()->hardwares.sensors.odo.setHeading(0,AngleUnit.DEGREES)
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
            shooter.setShooter(Constants.shooter40cm),
            light.setLightColor(0, 255, 0)
            );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.A)
        ).whenPressed(
            shooter.setShooter(Constants.shooter125cm),
            light.setLightColor(0, 0, 255));
    }
}
