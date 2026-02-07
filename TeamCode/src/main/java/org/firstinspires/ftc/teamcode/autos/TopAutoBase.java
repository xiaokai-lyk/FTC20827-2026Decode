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

public class TopAutoBase extends XKCommandOpmode {
    protected final double autoPanTargetX;
    protected final double autoPanTargetY;
    protected final double startDeg;
    protected final Pose startPose;
    protected final Pose shootPoseClose;
    protected final Pose intake1Pose;
    protected final Pose intake1PoseFin;
    protected final Pose intake2Pose;
    protected final Pose intake2PoseFin;
    protected final Pose openGatePos;
    protected final Pose openGateControl;
    protected final Pose endPose;


    public TopAutoBase(double autoPanTargetX, double autoPanTargetY, double startDeg,
                       Pose startPose, Pose shootPoseClose, Pose intake1Pose, Pose intake1PoseFin, Pose intake2Pose, Pose intake2PoseFin,
                       Pose openGatePos, Pose openGateControl, Pose endPose) {
        this.autoPanTargetX = autoPanTargetX;
        this.autoPanTargetY = autoPanTargetY;
        this.startDeg=startDeg;
        this.startPose=startPose;
        this.shootPoseClose = shootPoseClose;
        this.intake1Pose=intake1Pose;
        this.intake2Pose=intake2Pose;
        this.intake1PoseFin=intake1PoseFin;
        this.intake2PoseFin=intake2PoseFin;
        this.openGatePos = openGatePos;
        this.endPose=endPose;
        this.openGateControl=openGateControl;
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


    private Path shootLoaded, moveToRow1, grabRow1, openGate, moveToRow2, grabRow2, repeatOpenGate;
    private Supplier<PathChain> pathChainSupplier, moveToClose;

    public void buildPaths() {
        shootLoaded = new Path(new BezierLine(startPose, shootPoseClose));
        shootLoaded.setLinearHeadingInterpolation(startPose.getHeading(), shootPoseClose.getHeading());

        moveToRow1 = new Path(new BezierLine(shootPoseClose, intake1Pose));
        moveToRow1.setLinearHeadingInterpolation(shootPoseClose.getHeading(), intake1Pose.getHeading());

        grabRow1 = new Path(new BezierLine(intake1Pose,intake1PoseFin));
        grabRow1.setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1PoseFin.getHeading());

        openGate = new Path(new BezierCurve(intake1PoseFin,openGateControl, openGatePos));
        openGate.setLinearHeadingInterpolation(intake1PoseFin.getHeading(), openGatePos.getHeading());

        moveToClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, shootPoseClose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, shootPoseClose.getHeading(), 0.8))
                .build();

        moveToRow2 = new Path(new BezierLine(shootPoseClose,intake2Pose));
        moveToRow2.setLinearHeadingInterpolation(shootPoseClose.getHeading(),intake2Pose.getHeading());

        grabRow2 = new Path(new BezierLine(intake2Pose,intake2PoseFin));
        grabRow2.setLinearHeadingInterpolation(intake2Pose.getHeading(),intake2PoseFin.getHeading());

        repeatOpenGate = new Path(new BezierLine(shootPoseClose, openGatePos));
        repeatOpenGate.setLinearHeadingInterpolation(shootPoseClose.getHeading(), openGatePos.getHeading());


        pathChainSupplier = () -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, endPose)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endPose.getHeading(), 0.8))
            .build();
    }

    public void autoPathUpdate() {
        if (opmodeTimer.getElapsedTimeSeconds() > 28 && !leaveLine) {
            intake.stopIntake().schedule();
            gate.close().schedule();
            shooter.stopShooter().schedule();

            leaveLine = true;
            setPathState(-2);
        }

        switch (pathState) {
            case 0:
                follower.followPath(shootLoaded, true);
                setPathState(1);

                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    gate.open().schedule();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1.2){
                    gate.close().schedule();
                    follower.followPath(moveToRow1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(grabRow1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(moveToClose.get(), true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    gate.open().schedule();
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 1.2){
                    gate.close().schedule();
                    follower.followPath(moveToRow2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabRow2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(moveToClose.get(), true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    gate.open().schedule();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(pathChainSupplier.get(), true);
                    setPathState(-1);
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

        hardwares.sensors.odo.setHeading(startDeg, AngleUnit.DEGREES);

        pinpointDriverData = new PinpointDriverData(hardwares.sensors.odo);
        autoPan = new AutoPan(hardwares, autoPanTargetX, autoPanTargetY);

        autoPan.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        gate.close().schedule();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void onStart() {
        shooter.setShooterConfig(Shooter.shooter40cm).schedule();
        autoPan.setup();
        autoPan.setMode(AutoPan.Mode.HOLD);
        intake.startIntake().schedule();

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void run() {


        intake.startIntake().schedule();

        CommandScheduler.getInstance().run();
        pinpointDriverData.update();
        shooter.run();
        autoPan.run(pinpointDriverData);

        follower.update();
        autoPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("headingPan",autoPan.getTelemetryStatus().rawTargetAngle);

        telemetry.update();
    }
}
