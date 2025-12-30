package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoPan;
import org.firstinspires.ftc.teamcode.subsystems.OdoData;

@TeleOp(name="Chassis")
public class TeleopTest extends LinearOpMode {
    private AutoPan autoPan;
    private Hardwares hardwares;
    protected AutoPan.AutoPanCommand autoPanCommand;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwares = new Hardwares(hardwareMap);

        DcMotorEx pan = hardwares.motors.pan;

        DcMotorEx frontLeftMotor = hardwares.motors.mLeftFront;
        DcMotorEx backLeftMotor = hardwares.motors.mLeftRear;
        DcMotorEx frontRightMotor = hardwares.motors.mRightFront;
        DcMotorEx backRightMotor = hardwares.motors.mRightRear;

        waitForStart();
        autoPan=new AutoPan(hardwares);
        autoPanCommand = new AutoPan.AutoPanCommand(
                autoPan,
                hardwares.sensors.odo
        );

        CommandScheduler.getInstance().schedule(autoPanCommand);

        while (opModeIsActive()){

            CommandScheduler.getInstance().run();
            hardwares.sensors.odo.update();
            if(gamepad1.x){
                hardwares.sensors.odo.setPosition(new Pose2D(DistanceUnit.CM,-360.68,0, AngleUnit.DEGREES,0));
            }
            telemetry.addData("x",hardwares.sensors.odo.getPosX(DistanceUnit.CM));
            telemetry.addData("y",hardwares.sensors.odo.getPosY(DistanceUnit.CM));
            telemetry.addData("headings",hardwares.sensors.odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("headings+180",hardwares.sensors.odo.getHeading(AngleUnit.DEGREES)+180);

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double powerCoefficient = 1.0;

            frontLeftMotor.setPower(frontLeftPower * powerCoefficient);
            backLeftMotor.setPower(backLeftPower * powerCoefficient);
            frontRightMotor.setPower(frontRightPower * powerCoefficient);
            backRightMotor.setPower(backRightPower * powerCoefficient);

            telemetry.addData("pan position",pan.getCurrentPosition());
            telemetry.update();
        }
    }
}
