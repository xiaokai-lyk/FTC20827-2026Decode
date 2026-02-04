package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
    protected final double startDeg;

    protected final Pose startPose;
    protected final Pose cornerPose;
    protected final Pose endPose;

    public BottomAutoBase(double autoPanTargetX, double autoPanTargetY, double pedroTargetX, double pedroTargetY, double startDeg, Pose startPose, Pose cornerPose, Pose endPose) {
        this.autoPanTargetX = autoPanTargetX;
        this.autoPanTargetY = autoPanTargetY;
        this.pedroTargetX = pedroTargetX;
        this.pedroTargetY = pedroTargetY;
        this.startDeg = startDeg;
        this.startPose = startPose;
        this.cornerPose = cornerPose;
        this.endPose = endPose;
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


    private Path grabCorner, returnToStart;
    private Supplier<PathChain> pathChainSupplier;

    public void buildPaths() {
        grabCorner = new Path(new BezierLine(startPose, cornerPose));
        grabCorner.setLinearHeadingInterpolation(startPose.getHeading(), cornerPose.getHeading());

        returnToStart = new Path(new BezierLine(cornerPose, startPose));
        returnToStart.setLinearHeadingInterpolation(cornerPose.getHeading(), startPose.getHeading());

        pathChainSupplier = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, endPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endPose.getHeading(), 0.8))
                .build();
    }

    public void autoPathUpdate() {
        if (opmodeTimer.getElapsedTimeSeconds() > 27 && !leaveLine) {
            intake.stopIntake().schedule();
            gate.close().schedule();
            shooter.stopShooter().schedule();
            autoPan.setHoldAngle(0);
            follower.followPath(pathChainSupplier.get());
            leaveLine = true;
            setPathState(-1);
        }

        switch (pathState) {
            case 0:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    gate.open().schedule();
                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    gate.close().schedule();
                    follower.followPath(grabCorner, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5
                ) {
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

        double holdAngle = Math.toDegrees(Math.atan2(autoPanTargetY, autoPanTargetX)) - startDeg;
        autoPan.setHoldAngle(holdAngle);
        gate.close().schedule();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void onStart() {
        shooter.setShooterConfig(Shooter.shooterFar).schedule();
        autoPan.setup();
        intake.startIntake().schedule();

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void run() {
        pinpointDriverData.update();

        CommandScheduler.getInstance().run();
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
