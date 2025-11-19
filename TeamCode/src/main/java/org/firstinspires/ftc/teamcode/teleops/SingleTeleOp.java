package org.firstinspires.ftc.teamcode.teleops;



import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@TeleOp(name = "singleTeleOp", group = "teleops")
public class SingleTeleOp extends XKCommandOpmode {
    private Shooter shooter;
    private Hardwares hardwares;
    private Intake intake;
    private GamepadEx gamepad1;
    @Override
    public void initialize() {
        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);


        MecanumDrive drive = new MecanumDrive(hardwares);

        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(
                drive,
                gamepad1::getLeftX,
                gamepad1::getLeftY,
                gamepad1::getRightX,
                ()-> hardwares.sensors.odo.getHeading(AngleUnit.RADIANS),
                ()->1
        );

        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void onStart() {
        shooter.setShooter(Constants.shooterFar).schedule();
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        hardwares.sensors.odo.update();
        telemetry.addData("Heading", hardwares.sensors.odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("X", hardwares.sensors.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Y", hardwares.sensors.odo.getPosY(DistanceUnit.MM));
        telemetry.addLine("---");
        telemetry.addData("front left power", hardwares.motors.mFrontLeft.getPower());
        telemetry.addData("front right power", hardwares.motors.mFrontRight.getPower());
        telemetry.addData("back left power", hardwares.motors.mBackLeft.getPower());
        telemetry.addData("back right power", hardwares.motors.mBackRight.getPower());
        telemetry.update();
    }


    @Override
    public void functionalButtons() {
        new ButtonEx(
                () -> gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        ).whenPressed(intake.startIntake());

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(shooter.runPreShooter());

        new ButtonEx(
                ()-> gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        ).whenPressed(intake.stopIntake());

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(shooter.stopPreShooter());
    }
}
