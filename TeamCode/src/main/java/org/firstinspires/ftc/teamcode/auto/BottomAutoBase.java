package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoPan;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PinpointDriverData;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

import java.util.function.Supplier;

public class BottomAutoBase extends XKCommandOpmode {
    protected final double autoPanTargetX;
    protected final double autoPanTargetY;
    protected final double pedroTargetX;
    protected final double pedroTargetY;

    public BottomAutoBase(double autoPanTargetX, double autoPanTargetY, double pedroTargetX, double pedroTargetY) {
        this.autoPanTargetX = autoPanTargetX;
        this.autoPanTargetY = autoPanTargetY;
        this.pedroTargetX = pedroTargetX;
        this.pedroTargetY = pedroTargetY;
    }

    private Hardwares hardwares;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoPan autoPan;
    private PinpointDriverData pinpointDriverData;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private boolean leaveLine = false;

    private final Pose startPose = new Pose(89, 7, 0);
    private final Pose cornerPose = new Pose(135, 7, 0);
    private final Pose endPose = new Pose(100, 20, 0);

    private Path grabCorner, returnToStart;
    private Supplier<PathChain> pathChainSupplier;

    public void buildPaths() {
        grabCorner = new Path(new BezierLine(startPose, cornerPose));
        grabCorner.setLinearHeadingInterpolation(startPose.getHeading(), cornerPose.getHeading());

        returnToStart = new Path(new BezierLine(cornerPose, startPose));
        returnToStart.setLinearHeadingInterpolation(cornerPose.getHeading(), startPose.getHeading());

        pathChainSupplier = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower.getPose(), endPose)))
                .setLinearHeadingInterpolation(follower.getHeading(), endPose.getHeading())
                .build();
    }

    public void autoPathUpdate() {
        if (opmodeTimer.getElapsedTimeSeconds() > 27 && !leaveLine) {
            gate.close();
            follower.followPath(pathChainSupplier.get());
            leaveLine = true;
            setPathState(-1);
        }

        switch (pathState) {
            case 0:
                if (!follower.isBusy() && !autoPan.isPanBusy()) {
                    gate.open();
                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    gate.close();
                    follower.followPath(grabCorner, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(returnToStart, true);
                    setPathState(0);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();

        hardwares = new Hardwares(hardwareMap);
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        gate = new Gate(hardwares);
        pinpointDriverData = new PinpointDriverData(hardwares.sensors.odo);
        autoPan = new AutoPan(hardwares, autoPanTargetX, autoPanTargetY);

        autoPan.init();

        double holdAngle = Math.toDegrees(Math.atan2(pedroTargetX - startPose.getX(), pedroTargetY - startPose.getY()));
        autoPan.setHoldAngle(holdAngle);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void onStart() {
        shooter.setShooterConfig(Shooter.shooterFar);
        autoPan.setup();
        intake.startIntake();
        gate.close();

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void run() {
        pinpointDriverData.update();
        shooter.run();
        autoPan.run(pinpointDriverData);

        follower.update();
        autoPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
