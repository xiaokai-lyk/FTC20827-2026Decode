package org.firstinspires.ftc.teamcode.teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    protected final double startDeg;
    protected final Pose2D hpZonePose;


    public TeleOpBase(double targetX, double targetY, double startDeg, Pose2D hpZonePose) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.startDeg = startDeg;
        this.hpZonePose = hpZonePose;
    }

    private Hardwares hardwares;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoPan autoPan;
    private Drive drive;
    private PinpointDriverData pinpointDriverData;
    private GamepadEx gamepad1, gamepad2;
    private Drive.DriveCommand driveCommand;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.multipleTelemetry.addData("TargetX", targetX);
        this.multipleTelemetry.addData("TargetY", targetY);
        this.gamepad1 = new GamepadEx(super.gamepad1);
        this.gamepad2 = new GamepadEx(super.gamepad2);
        CommandScheduler.getInstance().reset();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        gate = new Gate(hardwares);

        hardwares.sensors.odo.setHeading(startDeg, AngleUnit.DEGREES);
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
        gate.close().schedule();
        autoPan.setup();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        this.pinpointDriverData.update();
        autoPan.run(this.pinpointDriverData);
        shooter.run();

        this.multipleTelemetry.addData("Heading (Rad)", pinpointDriverData.getHeadingRadians());
        this.multipleTelemetry.addData("Heading (Deg)", pinpointDriverData.getHeadingDegrees());
        this.multipleTelemetry.addData("X", pinpointDriverData.getRobotX());
        this.multipleTelemetry.addData("Y", pinpointDriverData.getRobotY());

        this.multipleTelemetry.addLine("---");
        double[] velocities = drive.getVelocities();
        this.multipleTelemetry.addData("use encoders", driveCommand.useEncoders);
        this.multipleTelemetry.addData("left front velocity", velocities[0]);
        this.multipleTelemetry.addData("right front velocity", velocities[1]);
        this.multipleTelemetry.addData("left rear velocity", velocities[2]);
        this.multipleTelemetry.addData("right rear velocity", velocities[3]);


        this.multipleTelemetry.addLine("---");
        AutoPan.TelemetryState panTelemetry = autoPan.getTelemetryStatus();
        this.multipleTelemetry.addData("pan mode", panTelemetry.mode);
        this.multipleTelemetry.addData("pan angle (deg)", panTelemetry.currentAngle);
        this.multipleTelemetry.addData("pan limit reached", panTelemetry.isLimitReached);

        this.multipleTelemetry.addLine("---");
        Shooter.TelemetryState shooterState = shooter.getTelemetryState();
        this.multipleTelemetry.addData("shooter left current", shooterState.leftCurrent);
        this.multipleTelemetry.addData("shooter right current", shooterState.rightCurrent);
        this.multipleTelemetry.addData("shooter left vel", shooterState.leftVelocity);
        this.multipleTelemetry.addData("shooter right vel", shooterState.rightVelocity);
        this.multipleTelemetry.addData("shooter pitch angle", shooterState.pitchAngle);
        this.multipleTelemetry.addData("shooter controller output", shooterState.controllerOutput);
        this.multipleTelemetry.addData("gate angle (deg)", gate.getAngleDeg());

        this.multipleTelemetry.addLine("---");
        this.multipleTelemetry.addData("gamepad 1 touch pad finger 1", gamepad1.gamepad.touchpad_finger_1);
        this.multipleTelemetry.addData("gamepad 1 touch pad finger 2", gamepad1.gamepad.touchpad_finger_2);

        this.multipleTelemetry.update();
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
                () -> gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)
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
                ()-> gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
                shooter.shooterIdle()
        );

        new ButtonEx(
                ()-> (gamepad1.gamepad.touchpadWasPressed() && gamepad1.gamepad.touchpad_finger_1 && gamepad1.gamepad.touchpad_finger_2)
        ).whenPressed(
                ()-> {
                    hardwares.sensors.odo.setPosition(this.hpZonePose);
                    gamepad1.gamepad.rumble(300);
                }
        );

        new ButtonEx(
                ()-> gamepad2.getButton(GamepadKeys.Button.Y)
        ).whenPressed(
                shooter.setShooterConfig(Shooter.shooter40cm)
        );

        new ButtonEx(
                ()->gamepad2.getButton(GamepadKeys.Button.A)
        ).whenPressed(
                shooter.setShooterConfig(Shooter.shooterNearTop)
        );

        new ButtonEx(
                ()->gamepad2.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                shooter.setShooterConfig(Shooter.shooterFar)
        );

        new ButtonEx(
                () -> gamepad2.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
                () -> autoPan.switchMode()
        );
    }
}
