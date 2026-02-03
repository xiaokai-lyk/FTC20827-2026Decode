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

public class TopAutoBase extends XKCommandOpmode {
    protected final double autoPanTargetX;
    protected final double autoPanTargetY;
    protected final Pose startPose;
    protected final Pose shootPose;
    protected final Pose intake1Pose;
    protected final Pose intake2Pose;
    protected final Pose endPose;
    protected final Pose intake1PoseFin;
    protected final Pose intake2PoseFin;
    protected final Pose opengatePrep;
    protected final Pose opengatePos;


    public TopAutoBase(double autoPanTargetX, double autoPanTargetY,
                       Pose startPose, Pose intake1Pose,Pose intake1PoseFin, Pose intake2Pose, Pose intake2PoseFin,
                       Pose shootPose, Pose opengatePrep, Pose opengatePos, Pose endPose) {
        this.autoPanTargetX = autoPanTargetX;
        this.autoPanTargetY = autoPanTargetY;
        this.startPose=startPose;
        this.shootPose=shootPose;
        this.intake1Pose=intake1Pose;
        this.intake2Pose=intake2Pose;
        this.intake1PoseFin=intake1PoseFin;
        this.intake2PoseFin=intake2PoseFin;
        this.opengatePrep=opengatePrep;
        this.opengatePos=opengatePos;
        this.endPose=endPose;
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


    private Path shootfirstballs, movetoball1, grabball1, movetoshoot1, movetoGate, openGate, movetoball2, grabball2, movetoshoot2;
    private Supplier<PathChain> pathChainSupplier;

    public void buildPaths() {
        shootfirstballs = new Path(new BezierLine(startPose, shootPose));
        shootfirstballs.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        movetoball1 = new Path(new BezierLine(shootPose, intake1Pose));
        movetoball1.setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading());

        grabball1 = new Path(new BezierLine(intake1Pose,intake1PoseFin));
        grabball1.setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1PoseFin.getHeading());

        movetoGate = new Path(new BezierLine(intake1PoseFin,opengatePrep));
        movetoGate.setLinearHeadingInterpolation(intake1PoseFin.getHeading(),opengatePrep.getHeading());

        openGate = new Path(new BezierLine(opengatePrep,opengatePos));
        openGate.setLinearHeadingInterpolation(opengatePrep.getHeading(),opengatePos.getHeading());

        movetoshoot1 = new Path(new BezierLine(opengatePos,shootPose));
        movetoshoot1.setLinearHeadingInterpolation(opengatePos.getHeading(),shootPose.getHeading());

        movetoball2 = new Path(new BezierLine(shootPose,intake2Pose));
        movetoball2.setLinearHeadingInterpolation(shootPose.getHeading(),intake2Pose.getHeading());

        grabball2 = new Path(new BezierLine(intake2Pose,intake2PoseFin));
        grabball2.setLinearHeadingInterpolation(intake2Pose.getHeading(),intake2PoseFin.getHeading());

        movetoshoot2 = new Path(new BezierLine(intake2PoseFin,shootPose));
        movetoball2.setLinearHeadingInterpolation(intake2PoseFin.getHeading(),shootPose.getHeading());

        pathChainSupplier = () -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, endPose)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endPose.getHeading(), 0.8))
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
                gate.close();
                if (!follower.isBusy()) {
                    follower.followPath(shootfirstballs, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy() && !autoPan.isPanBusy()) {
                    gate.open();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    gate.close();
                    follower.followPath(movetoball1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabball1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(movetoGate, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(openGate, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(movetoshoot1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && !autoPan.isPanBusy()){
                    gate.open();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >2) {
                    gate.close();
                    follower.followPath(movetoball2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabball2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(movetoshoot2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() && !autoPan.isPanBusy()){
                    gate.open();
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
        pinpointDriverData = new PinpointDriverData(hardwares.sensors.odo);
        autoPan = new AutoPan(hardwares, autoPanTargetX, autoPanTargetY);

        autoPan.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void onStart() {
        shooter.setShooterConfig(Shooter.shooter40cm);
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
