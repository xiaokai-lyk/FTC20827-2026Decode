package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@Autonomous(name="FromBottomToTopRed", group="autos")
public class FromBottomToTopRed extends XKCommandOpmode {
    private Hardwares hardwares;
    private Drive drive;
    private AutoDrive autoDrive;
    private AdaptivePoseController adaptiveController;
    private Shooter shooter;
    private Intake intake;
    private OdometerData odo;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private enum AutoStep {
        WAIT_FOR_TIMING,
        DRIVE_FORWARD,
        GOTO_SHOOTING_POS,
        SHOOT,
        STOP_SYSTEMS,
        COMPLETE
    }

    private AutoStep currentStep;
    private long stepStartTime;

    @Override
    public void initialize() {
        hardwares = new Hardwares(hardwareMap);
        drive = new Drive(hardwares);
        autoDrive = new AutoDrive();
        adaptiveController = Constants.PID.newPoseController();
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        odo = new OdometerData(hardwares.sensors.odo);

        // Set starting pose at origin facing 0 degrees
        hardwares.sensors.odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
        telemetry.addData("Auto Status", "Initialized");
    }

    @Override
    public void onStart() {
        currentStep = AutoStep.WAIT_FOR_TIMING;
        stepStartTime = System.currentTimeMillis();
        shooter.blockBallPass().schedule();
        shooter.setShooter(Constants.shooter40cm).schedule();
        telemetry.addData("Auto Status", "Started");
    }

    @Override
    public void run() {
        hardwares.sensors.odo.update();
        odo = new OdometerData(hardwares.sensors.odo);

        executeCurrentStep();
        scheduler.run();
        updateTelemetry();
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case WAIT_FOR_TIMING:
                if (getElapsedSeconds() >= 5.0) {
                    transitionToNextStep(AutoStep.DRIVE_FORWARD);
                }
                break;
            case DRIVE_FORWARD:
                driveForward();
                break;
            case GOTO_SHOOTING_POS:
                moveToShootingPos();
                break;
            case SHOOT:
                shoot();
                break;
            case STOP_SYSTEMS:
                stopSystems();
                break;
            case COMPLETE:
                break;
        }
    }

    private void driveForward() {
        adaptiveController.positionDeadbandCm = 3;
        adaptiveController.headingDeadbandRad = Math.toRadians(3);

        // Target forward 500 cm from origin on Y axis (facing 0 degrees means +Y is forward)
        AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                295,              // x target (cm)
                0,            // y target (cm)
                0,              // heading target (deg)
                odo,
                1.0,
                false
        );
        if (out.atPosition && out.atHeading || getElapsedSeconds() > 5.0) {
            adaptiveController.resetDeadbands();
            transitionToNextStep(AutoStep.GOTO_SHOOTING_POS);
        }
    }

    private void moveToShootingPos() {
        adaptiveController.headingDeadbandRad = Math.toRadians(3);

        // Keep position at (0, 500) but rotate to 90 degrees (left turn)
        AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                295,
                20,
                -90,
                odo,
                0.7,
                true
        );
        if (out.atHeading || getElapsedSeconds() > 3.0) {
            adaptiveController.resetDeadbands();
            transitionToNextStep(AutoStep.SHOOT);
        }
    }

    private void shoot() {
        // Spin up shooter and feed a ball
        shooter.allowBallPassClose().schedule();
        intake.startIntake(1).schedule();

        // Hold heading and position while shooting
        autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                295,
                -10,
                -90,
                odo,
                0.0,
                false
        );

        // After a short time, stop systems and complete
        if (getElapsedSeconds() > 2.0) {
            transitionToNextStep(AutoStep.STOP_SYSTEMS);
        }
    }

    private void stopSystems() {
        // Stop shooter and intake, then complete (following TopRouteBlue style)
        shooter.setShooter(Constants.shooterStop).schedule();
        intake.stopIntake().schedule();
        transitionToNextStep(AutoStep.COMPLETE);
    }

    private void transitionToNextStep(AutoStep next) {
        currentStep = next;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", next.toString());
    }

    private double getElapsedSeconds() {
        return (System.currentTimeMillis() - stepStartTime) / 1000.0;
    }

    private void updateTelemetry() {
        telemetry.addData("Current Step", currentStep);
        telemetry.addData("Step Elapsed Time", String.format("%.1f sec", getElapsedSeconds()));
        telemetry.addData("heading deadband", adaptiveController.headingDeadbandRad);
        telemetry.addData("distance deadband", adaptiveController.positionDeadbandCm);
        telemetry.update();
    }
}
