package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@Autonomous(name = "Test", group = "autos")
public class Test extends XKCommandOpmode {
    Hardwares hardwares;
    Drive drive;
    AutoDrive autoDrive;
    AdaptivePoseController adaptiveController;
    Shooter shooter;
    Intake intake;

    int step = 0;

    @Override
    public void onStart() {

    }

    @Override public void run() {
        hardwares.sensors.odo.update();
        OdometerData odom = new OdometerData(hardwares.sensors.odo);
        if(step ==0){
            shooter.blockBallPass().schedule();
            shooter.setShooter(Constants.shooter125cm).schedule();
            intake.startIntake(false).schedule();
            AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                -100,
                0,
                0,
                odom,
                1,
                true
            );
            if(out.atPosition && out.atHeading && getRuntime() > 7){
                step = 1;
            }
        }
        if(step == 1){
            shooter.allowBallPass().schedule();
            intake.startIntake(true).schedule();
            if(getRuntime() > 15)step = 2;
        }
        if(step == 2){
            shooter.blockBallPass().schedule();
            AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                -100,
                -50,
                0,
                odom,
                1,
                true
            );
            if(out.atPosition && out.atHeading && getRuntime() > 3){
                step = 3;
            }
        }
        if(step == 3){
            shooter.setShooter(Constants.shooterStop).schedule();
            intake.stopIntake().schedule();
        }
        CommandScheduler.getInstance().run();
        telemetry.addData("current step", step);
        telemetry.update();
    }

    @Override
    public void initialize() {
        hardwares = new Hardwares(hardwareMap);
        drive = new Drive(hardwares);
        autoDrive = new AutoDrive();
        adaptiveController = Constants.PID.newPoseController();
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
    }
}
