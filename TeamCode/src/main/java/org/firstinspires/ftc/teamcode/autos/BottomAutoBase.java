package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    protected final double startDeg;

    protected final Pose startPose;
    protected final Pose cornerPose;
    protected final Pose thirdRowPose;
    protected final Pose thirdRowCtrl;
    protected final Pose endPose;

    public BottomAutoBase(double autoPanTargetX, double autoPanTargetY, double startDeg, Pose startPose, Pose cornerPose, Pose thirdRowPose, Pose thirdRowCtrl, Pose endPose) {
        this.autoPanTargetX = autoPanTargetX;
        this.autoPanTargetY = autoPanTargetY;
        this.startDeg = startDeg;
        this.startPose = startPose;
        this.cornerPose = cornerPose;
        this.thirdRowPose = thirdRowPose;
        this.thirdRowCtrl = thirdRowCtrl;
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
    private boolean shooterAccelerating = true;
    private boolean leaveLine = false;


    private Path grabCorner, cornerReturnToStart, grabThirdRow, thirdRowReturnToStart;
    private Supplier<PathChain> pathChainSupplier;

    public void buildPaths() {
        grabCorner = new Path(new BezierLine(startPose, cornerPose));
        grabCorner.setLinearHeadingInterpolation(startPose.getHeading(), cornerPose.getHeading());

        cornerReturnToStart = new Path(new BezierLine(cornerPose, startPose));
        cornerReturnToStart.setLinearHeadingInterpolation(cornerPose.getHeading(), startPose.getHeading());

        grabThirdRow = new Path(new BezierCurve(startPose, thirdRowCtrl, thirdRowPose));
        grabThirdRow.setLinearHeadingInterpolation(startPose.getHeading(), thirdRowPose.getHeading());

        thirdRowReturnToStart = new Path(new BezierLine(thirdRowPose, startPose));
        thirdRowReturnToStart.setLinearHeadingInterpolation(thirdRowPose.getHeading(), startPose.getHeading());
        thirdRowReturnToStart.setBrakingStrength(0.7);

        pathChainSupplier = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, endPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endPose.getHeading(), 0.8))
                .setBrakingStrength(0.9)
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
                if (shooterAccelerating) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        shooterAccelerating = false;
                        setPathState(1);
                    }
                } else {
                    if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 0.4) {
                        intake.stopIntake().schedule();
                    } else {
                        intake.startIntake().schedule();
                    }

                    if (!follower.isBusy()) {
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    gate.open().schedule();
                    setPathState(2);
                }
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gate.close().schedule();
                    follower.followPath(grabCorner, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(cornerReturnToStart, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 0.4) {
                    intake.stopIntake().schedule();
                } else {
                    intake.startIntake().schedule();
                }

                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    gate.open().schedule();
                    setPathState(6);
                }
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gate.close().schedule();
                    follower.followPath(grabThirdRow, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(thirdRowReturnToStart, true);
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

        hardwares.sensors.odo.setHeading(startDeg, AngleUnit.DEGREES);

        autoPan.init();

        double holdAngle = Math.toDegrees(Math.atan2(autoPanTargetY, autoPanTargetX)) - startDeg;
        autoPan.setHoldAngle(holdAngle);

        gate.close().schedule();

        CommandScheduler.getInstance().run();

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
        telemetry.addData("path timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
