package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OdoData;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@TeleOp(name = "DriveTeleOp", group = "teleop")
public class DriveTeleOp extends XKCommandOpmode {
//    private Shooter shooter;
    private Hardwares hardwares;
    private Intake intake;
//    private Gate gate;
    private GamepadEx gamepad1;
    protected Drive.DriveCommand driveCommand;
    private FtcDashboard dashboard;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();

        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
//        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
//        gate = new Gate(hardwares);

        Drive drive = new Drive(hardwares);


        driveCommand = new Drive.DriveCommand(
                drive,
                () -> gamepad1.getLeftX(),
                () -> gamepad1.getLeftY(),
                () -> -gamepad1.getRightX(),
                () -> new OdoData(hardwares.sensors.odo),
                1,
                true,
                false
        );

        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void onStart() {}

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        hardwares.sensors.odo.update();
        telemetry.addData("Heading (Rad)", hardwares.sensors.odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Heading (Deg)", hardwares.sensors.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X", hardwares.sensors.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Y", hardwares.sensors.odo.getPosY(DistanceUnit.MM));
        telemetry.addLine("---");

        telemetry.addData("drive mode", hardwares.motors.mLeftFront.getMode().toString());
        telemetry.addData("left front velocity", hardwares.motors.mLeftFront.getVelocity());
        telemetry.addData("right front velocity", hardwares.motors.mRightFront.getVelocity());
        telemetry.addData("left rear velocity", hardwares.motors.mLeftRear.getVelocity());
        telemetry.addData("right rear velocity", hardwares.motors.mRightRear.getVelocity());

        telemetry.addLine("---");
        telemetry.addData("X damping", driveCommand.dampedX - gamepad1.getLeftX());
        telemetry.addData("Y damping", driveCommand.dampedY - gamepad1.getLeftY());
        telemetry.addData("Rotate damping", driveCommand.dampedRotate - (-gamepad1.getRightX()));

//        dashboard.getTelemetry().addData("front shooter velocity", hardwares.motors.shooterFront.getVelocity());
//        dashboard.getTelemetry().addData("back shooter velocity", hardwares.motors.shooterBack.getVelocity());

        // This part is for tuning shooter PID
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("shooter/front_velocity", hardwares.motors.shooterFront.getVelocity());
//        packet.put("shooter/back_velocity", hardwares.motors.shooterBack.getVelocity());
//        packet.put("shooter/front_target", Constants.shooter40cm.frontVelocity);
//        packet.put("shooter/back_target", Constants.shooter40cm.backVelocity);
//        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }


    @Override
    public void functionalButtons() {
        ParallelCommandGroup enterRunningMode = new ParallelCommandGroup(
//                gate.close(),
                intake.stopIntake()
        );
        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
//                gate.close(),
                intake.startIntake()
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                ()-> gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        ).whenPressed(
//                gate.open(),
                intake.startIntake()
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
//                shooter.stopShooter()
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenPressed(
                ()->hardwares.sensors.odo.setHeading(0,AngleUnit.DEGREES)
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.Y)
        ).whenPressed(
//                shooter.setShooter(Shooter.shooter40cm)
        );

        new ButtonEx(
                ()->gamepad1.getButton(GamepadKeys.Button.A)
        ).whenPressed(
//                shooter.setShooter(Shooter.shooter125cm)
        );

        new ButtonEx(
                ()->gamepad1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
//                shooter.setShooter(Shooter.shooter250cm)
        );
    }
}