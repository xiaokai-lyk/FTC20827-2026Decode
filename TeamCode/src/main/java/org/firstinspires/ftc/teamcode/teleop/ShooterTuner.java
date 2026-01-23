package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@Config
@TeleOp(name = "Shooter Tuner", group = "Tests")
public class ShooterTuner extends XKCommandOpmode {
    public static double SHOOTER_TARGET_VEL = 2200;
    public static double PITCH_TARGET_ANGLE = 50;
    private Hardwares hardwares;
    private Shooter shooter;
    private Intake intake;
//    private Gate gate;
    private GamepadEx gamepad1;
    private FtcDashboard dashboard;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();

        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);

//        CommandScheduler.getInstance().schedule(shooter.setPitch(() -> PITCH_TARGET_ANGLE));
    }

    @Override
    public void onStart() {}

    @Override
    public void run() {
        CommandScheduler.getInstance().run();



        Shooter.TelemetryState shooterState = shooter.getTelemetryState();
        telemetry.addData("left shooter vel", shooterState.leftVelocity);
        telemetry.addData("right shooter vel", shooterState.rightVelocity);
        telemetry.addData("left shooter current", shooterState.leftCurrent);
        telemetry.addData("right shooter current", shooterState.rightCurrent);
        telemetry.addData("pitch angle", shooterState.pitchAngle);
        telemetry.addData("target vel", SHOOTER_TARGET_VEL);
        dashboard.getTelemetry().addData("left shooter vel", shooterState.leftVelocity);
        dashboard.getTelemetry().addData("right shooter vel", shooterState.rightVelocity);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooter/left_velocity", shooterState.leftVelocity);
        packet.put("shooter/right_velocity", shooterState.rightVelocity);
        packet.put("shooter/left_target", SHOOTER_TARGET_VEL);
        packet.put("shooter/right_target", SHOOTER_TARGET_VEL);
        dashboard.sendTelemetryPacket(packet);

        dashboard.getTelemetry().update();
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
                ()-> gamepad1.getButton(GamepadKeys.Button.A)
        ).whenPressed(
                shooter.stopShooter()
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
                new InstantCommand(()-> {
                    SHOOTER_TARGET_VEL += 50;
                })
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
                new InstantCommand(()-> {
                    SHOOTER_TARGET_VEL -= 50;
                })
        );

        new ButtonEx(
                ()-> gamepad1.getButton(GamepadKeys.Button.Y)
        ).whenPressed(
                shooter.setShooter(() -> SHOOTER_TARGET_VEL)
        );
    }
}
