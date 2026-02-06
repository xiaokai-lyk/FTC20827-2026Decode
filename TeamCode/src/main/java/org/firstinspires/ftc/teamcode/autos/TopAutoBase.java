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
    protected final Pose shootPose1;
    protected final Pose shootPose2;
    protected final Pose intake1Pose;
    protected final Pose intake2Pose;
    protected final Pose endPose;
    protected final Pose intake1PoseFin;
    protected final Pose intake2PoseFin;
    protected final Pose opengatePos;
    protected final Pose openGateControl;


    public TopAutoBase(double autoPanTargetX, double autoPanTargetY, double startDeg,
                       Pose startPose, Pose intake1Pose,Pose intake1PoseFin, Pose intake2Pose, Pose intake2PoseFin,
                       Pose shootPose1,Pose shootPose2, Pose opengatePos, Pose openGateControl, Pose endPose) {
        this.autoPanTargetX = autoPanTargetX;
        this.autoPanTargetY = autoPanTargetY;
        this.startDeg=startDeg;
        this.startPose=startPose;
        this.shootPose1 =shootPose1;
        this.shootPose2= shootPose2;
        this.intake1Pose=intake1Pose;
        this.intake2Pose=intake2Pose;
        this.intake1PoseFin=intake1PoseFin;
        this.intake2PoseFin=intake2PoseFin;
        this.opengatePos=opengatePos;
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


    private Path shootfirstballs, movetoball1, grabball1, movetoshoot1, openGate, movetoball2, grabball2, movetoshoot2, repeatOpenGate;
    private Supplier<PathChain> pathChainSupplier;

    public void buildPaths() {
        shootfirstballs = new Path(new BezierLine(startPose, shootPose1));
        shootfirstballs.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());

        movetoball1 = new Path(new BezierLine(shootPose1, intake1Pose));
        movetoball1.setLinearHeadingInterpolation(shootPose1.getHeading(), intake1Pose.getHeading());

        grabball1 = new Path(new BezierLine(intake1Pose,intake1PoseFin));
        grabball1.setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1PoseFin.getHeading());
//
//        movetoGate = new Path(new BezierLine(intake1PoseFin,opengatePrep));
//        movetoGate.setLinearHeadingInterpolation(intake1PoseFin.getHeading(),opengatePrep.getHeading());

        openGate = new Path(new BezierCurve(intake1PoseFin,openGateControl,opengatePos));
        openGate.setLinearHeadingInterpolation(intake1PoseFin.getHeading(),opengatePos.getHeading());

        movetoshoot1 = new Path(new BezierLine(opengatePos, shootPose2));
        movetoshoot1.setLinearHeadingInterpolation(opengatePos.getHeading(), shootPose2.getHeading());

        movetoball2 = new Path(new BezierLine(shootPose2,intake2Pose));
        movetoball2.setLinearHeadingInterpolation(shootPose2.getHeading(),intake2Pose.getHeading());

        grabball2 = new Path(new BezierLine(intake2Pose,intake2PoseFin));
        grabball2.setLinearHeadingInterpolation(intake2Pose.getHeading(),intake2PoseFin.getHeading());

        movetoshoot2 = new Path(new BezierLine(intake2PoseFin, shootPose2));
        movetoshoot2.setLinearHeadingInterpolation(intake2PoseFin.getHeading(), shootPose2.getHeading());

        repeatOpenGate = new Path(new BezierLine(shootPose2,opengatePos));
        repeatOpenGate.setLinearHeadingInterpolation(shootPose2.getHeading(),opengatePos.getHeading());


        pathChainSupplier = () -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, endPose)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endPose.getHeading(), 0.8))
            .build();
    }

    public void autoPathUpdate() {
        if (opmodeTimer.getElapsedTimeSeconds() > 28 && !leaveLine) {
            gate.close().schedule();
            follower.followPath(pathChainSupplier.get());
            leaveLine = true;
            setPathState(-1);
        }

        switch (pathState) {
            case 0:
                gate.close().schedule();
                if (!follower.isBusy()) {
                    follower.followPath(shootfirstballs, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy() && !autoPan.isPanBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    gate.open().schedule();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.5){
                    gate.close().schedule();
                    shooter.setShooterConfig(Shooter.shooterNearTop).schedule();
                    follower.followPath(movetoball1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabball1, true);
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
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>3) {
                    follower.followPath(movetoshoot1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && !autoPan.isPanBusy()){
                    gate.open().schedule();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >2) {
                    gate.close().schedule();
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
                    gate.open().schedule();
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4){
                    gate.close().schedule();
                    follower.followPath(repeatOpenGate);
                    setPathState(13);
                }
            case 13:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4){
                    follower.followPath(pathChainSupplier.get());
                    setPathState(-1);
                }
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
        gate.close().schedule();
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
