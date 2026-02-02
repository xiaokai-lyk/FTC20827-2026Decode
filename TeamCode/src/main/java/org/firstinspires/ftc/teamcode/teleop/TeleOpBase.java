package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoPan;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PinpointDriverData;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;


public class TeleOpBase extends XKCommandOpmode {
    protected final double targetX;
    protected final double targetY;


    public TeleOpBase(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }

    private Hardwares hardwares;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoPan autoPan;
    private Drive drive;
    private PinpointDriverData pinpointDriverData;
    private GamepadEx gamepad1;
    private Drive.DriveCommand driveCommand;

    @Override
    public void initialize() {
        telemetry.addData("TargetX", targetX);
        telemetry.addData("TargetY", targetY);
        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().reset();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        gate = new Gate(hardwares);
        this.pinpointDriverData = new PinpointDriverData(hardwares.sensors.odo);

        autoPan = new AutoPan(hardwares, targetX, targetY);
        autoPan.init();

        drive = new Drive(hardwares);



        driveCommand = new Drive.DriveCommand(
                drive,
                () -> gamepad1.getLeftX(),
                () -> gamepad1.getLeftY(),
                () -> -gamepad1.getRightX(),
                () -> pinpointDriverData,
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

        this.pinpointDriverData.update();
        autoPan.run(this.pinpointDriverData);
        shooter.run();

        telemetry.addData("Heading (Rad)", pinpointDriverData.getHeadingRadians());
        telemetry.addData("Heading (Deg)", pinpointDriverData.getHeadingDegrees());
        telemetry.addData("X", pinpointDriverData.getRobotX());
        telemetry.addData("Y", pinpointDriverData.getRobotY());

        telemetry.addLine("---");
        double[] velocities = drive.getVelocities();
        telemetry.addData("use encoders", driveCommand.useEncoders);
        telemetry.addData("left front velocity", velocities[0]);
        telemetry.addData("right front velocity", velocities[1]);
        telemetry.addData("left rear velocity", velocities[2]);
        telemetry.addData("right rear velocity", velocities[3]);


        telemetry.addLine("---");
        AutoPan.TelemetryState panTelemetry = autoPan.getTelemetryStatus();
        telemetry.addData("pan mode", panTelemetry.mode);
        telemetry.addData("pan angle (deg)", panTelemetry.currentAngle);
        telemetry.addData("pan limit reached", panTelemetry.isLimitReached);

        telemetry.addLine("---");
        Shooter.TelemetryState shooterState = shooter.getTelemetryState();
        telemetry.addData("shooter left current", shooterState.leftCurrent);
        telemetry.addData("shooter right current", shooterState.rightCurrent);
        telemetry.addData("shooter left vel", shooterState.leftVelocity);
        telemetry.addData("shooter right vel", shooterState.rightVelocity);
        telemetry.addData("shooter target vel", shooterState.pitchAngle);
        telemetry.addData("shooter controller output", shooterState.controllerOutput);
        telemetry.addData("gate angle (deg)", gate.getAngleDeg());

        telemetry.update();
    }


    @Override
    public void functionalButtons() {
        /*
         键位：

         - 左肩扣（LEFT_BUMPER）：
             * 按下：gate.close(), intake.startIntake()
             * 释放：enterRunningMode（并行执行 gate.close() 和 intake.stopIntake()）

         - 左扳机（LEFT_TRIGGER，阈值 > 0.5）：
             * 按下：gate.open(), intake.startIntake()
             * 释放：enterRunningMode（并行执行 gate.close() 和 intake.stopIntake()）

         - 十字键下（DPAD_DOWN）：
             * 按下：shooter 怠速

         - 右摇杆按下（RIGHT_STICK_BUTTON）：
             * 按下：重置机器人当前方向为 0 度

         - Y 按钮：
             * 按下：40cm 发射

         - A 按钮：
             * 按下：125cm 发射

         - B 按钮：
             * 按下：250cm 发射

         - 十字键上（DPAD_UP）：
             * 按下：切换云台模式
        */

        ParallelCommandGroup enterRunningMode = new ParallelCommandGroup(
                gate.close(),
                intake.stopIntake()
        );
        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
                gate.close(),
                intake.startIntake()
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                ()-> gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        ).whenPressed(
                gate.open(),
                intake.startIntake()
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
                shooter.shooterIdle()
        );

        new ButtonEx(
                ()-> gamepad1.gamepad.touchpadWasPressed()
        ).whenPressed(
                ()->hardwares.sensors.odo.setHeading(0,AngleUnit.DEGREES)
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.Y)
        ).whenPressed(
                shooter.setShooterConfig(Shooter.shooter40cm)
        );

        new ButtonEx(
                ()->gamepad1.getButton(GamepadKeys.Button.A)
        ).whenPressed(
                shooter.setShooterConfig(Shooter.shooterNearTop)
        );

        new ButtonEx(
                ()->gamepad1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                shooter.setShooterConfig(Shooter.shooterFar)
        );

        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
                () -> autoPan.setMode()
        );
    }
}
