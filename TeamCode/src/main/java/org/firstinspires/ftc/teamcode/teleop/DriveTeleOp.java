package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoPan;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OdoData;
import org.firstinspires.ftc.teamcode.legacy.PanLucas;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends XKCommandOpmode {
    private Hardwares hardwares;
//    private Shooter shooter;
    private Intake intake;
//    private Gate gate;
    private AutoPan autoPan;
    private Drive drive;
    private OdoData odoData;
    private GamepadEx gamepad1;
    private Drive.DriveCommand driveCommand;
    private PanLucas.AutoPanCommand autoPanCommand;

    @Override
    public void initialize() {
        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
//        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
//        gate = new Gate(hardwares);
        odoData = new OdoData(hardwares.sensors.odo);

        autoPan = new AutoPan(hardwares, 100.0, 0.0);

        drive = new Drive(hardwares);

        driveCommand = new Drive.DriveCommand(
                drive,
                () -> gamepad1.getLeftX(),
                () -> gamepad1.getLeftY(),
                () -> -gamepad1.getRightX(),
                () -> odoData,
                1,
                true,
                false
        );

        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void onStart() {
        autoPan.setup();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        OdoData odoDataPan = new OdoData(hardwares.sensors.odo);

        autoPan.run(odoDataPan);

        hardwares.sensors.odo.update();
        telemetry.addData("Heading (Rad)", odoData.getHeadingRadians());
        telemetry.addData("Heading (Deg)", odoData.getHeadingDegrees());
        telemetry.addData("X", odoData.getRobotX());
        telemetry.addData("Y", odoData.getRobotY());

        telemetry.addLine("---");
        double[] velocities = drive.getVelocities();
        telemetry.addData("use encoders", driveCommand.useEncoders);
        telemetry.addData("left front velocity", velocities[0]);
        telemetry.addData("right front velocity", velocities[1]);
        telemetry.addData("left rear velocity", velocities[2]);
        telemetry.addData("right rear velocity", velocities[3]);

//        telemetry.addLine("---");
//        telemetry.addData("X damping", driveCommand.dampedX - gamepad1.getLeftX());
//        telemetry.addData("Y damping", driveCommand.dampedY - gamepad1.getLeftY());
//        telemetry.addData("Rotate damping", driveCommand.dampedRotate - (-gamepad1.getRightX()));

        telemetry.addLine("---");
        AutoPan.TelemetryState panTelemetry = autoPan.getTelemetryStatus();
        telemetry.addData("pan angle (deg)", panTelemetry.currentAngle);
//        telemetry.addData("gate angle (deg)", gate.getAngleDeg());
//        telemetry.addData("pitch angle", shooter.getPitchAngle());

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

//        new ButtonEx(
//                ()-> gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)
//        ).whenPressed(
//                shooter.stopShooter()
//        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenPressed(
                ()->hardwares.sensors.odo.setHeading(0,AngleUnit.DEGREES)
        );

//        new ButtonEx(
//                ()-> gamepad1.getButton(GamepadKeys.Button.Y)
//        ).whenPressed(
//                shooter.setShooterAndPitch(Shooter.shooter40cm)
//        );
//
//        new ButtonEx(
//                ()->gamepad1.getButton(GamepadKeys.Button.A)
//        ).whenPressed(
//                shooter.setShooterAndPitch(Shooter.shooter125cm)
//        );
//
//        new ButtonEx(
//                ()->gamepad1.getButton(GamepadKeys.Button.B)
//        ).whenPressed(
//                shooter.setShooterAndPitch(Shooter.shooter250cm)
//        );

        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
                () -> autoPan.setMode()
        );
    }
}