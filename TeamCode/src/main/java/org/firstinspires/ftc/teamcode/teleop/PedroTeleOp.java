package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OdoData;
import org.firstinspires.ftc.teamcode.legacy.PanLucas;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@TeleOp(name = "Pedro TeleOp", group = "TeleOp")
public class PedroTeleOp extends XKCommandOpmode {
    private Hardwares hardwares;
//    private Shooter shooter;
    private Intake intake;
    //    private Gate gate;
    private PanLucas panLucas;
    private Drive drive;
    private PanLucas.AutoPanCommand autoPanCommand;
    private OdoData odoData;
    private GamepadEx gamepad1;
    private Follower follower;
    public static Pose startingPose = new Pose(7, 9, 90);
    private boolean autoDrive;
    private Supplier<PathChain> pathChainSupplier;

    @Override
    public void initialize() {
        this.gamepad1 = new GamepadEx(super.gamepad1);
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
//        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
//        gate = new Gate(hardwares);
        drive = new Drive(hardwares);
        panLucas = new PanLucas(hardwares);

        autoPanCommand = new PanLucas.AutoPanCommand(panLucas, () -> odoData);

        CommandScheduler.getInstance().schedule(autoPanCommand);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        pathChainSupplier = () -> follower.pathBuilder() // Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void onStart() {
        follower.startTeleOpDrive();
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        hardwares.sensors.odo.update();
        hardwares.sensors.odo.update();
        telemetry.addData("Heading (Rad)", odoData.getHeadingRadians());
        telemetry.addData("Heading (Deg)", odoData.getHeadingDegrees());
        telemetry.addData("X", odoData.getRobotX());
        telemetry.addData("Y", odoData.getRobotY());

        telemetry.addLine("---");

        double[] velocities = drive.getVelocities();
        telemetry.addData("left front velocity", velocities[0]);
        telemetry.addData("right front velocity", velocities[1]);
        telemetry.addData("left rear velocity", velocities[2]);
        telemetry.addData("right rear velocity", velocities[3]);

        telemetry.addLine("---");
        telemetry.addData("pan angle (deg)", panLucas.getCurrentPanDeg());
//        telemetry.addData("gate angle (deg)", gate.getAngleDeg());
//        telemetry.addData("pitch angle", shooter.getPitchAngle());

        telemetry.update();

        follower.update();

        if (!autoDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.getLeftY(),
                    -gamepad1.getLeftX(),
                    -gamepad1.getRightX(),
                    true
            );
        }
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
                () -> gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        ).whenPressed(
//                gate.open(),
                intake.startIntake()
        ).whenReleased(enterRunningMode);

        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
//                shooter.stopShooter(),
                stopAutoDrive()
        );

        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenPressed(
//                shooter.stopShooter(),
                stopAutoDrive()
        );

        new ButtonEx(
                () -> gamepad1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenPressed(
                () -> hardwares.sensors.odo.setHeading(0,AngleUnit.DEGREES)
        );

//        new ButtonEx(
//                () -> gamepad1.getButton(GamepadKeys.Button.Y)
//        ).whenPressed(
//                shooter.setShooterAndPitch(Shooter.shooter40cm)
//        );
//
//        new ButtonEx(
//                () ->gamepad1.getButton(GamepadKeys.Button.A)
//        ).whenPressed(
//                shooter.setShooterAndPitch(Shooter.shooter125cm)
//        );
//
//        new ButtonEx(
//                () ->gamepad1.getButton(GamepadKeys.Button.B)
//        ).whenPressed(
//                shooter.setShooterAndPitch(Shooter.shooter250cm)
//        );
    }

    private InstantCommand startAutoDrive() {
        return new InstantCommand(() -> {
            autoDrive = true;
            follower.followPath(pathChainSupplier.get());
        });
    }

    private InstantCommand stopAutoDrive() {
        return new InstantCommand(() -> {
            autoDrive = false;
            follower.startTeleOpDrive();
        });
    }
}
