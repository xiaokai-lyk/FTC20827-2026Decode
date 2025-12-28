package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


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
    public Hardwares.Sensors sensors;
    public Hardwares.Motors motors;
    @Override
    public void runOpMode() throws InterruptedException {
        sensors=hardwares.sensors;
        motors=hardwares.motors;
        DcMotorEx pan = hardwares.motors.pan;

        hardwares = new Hardwares(hardwareMap);
        OdoData odo=new OdoData(sensors.odo);

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftRear");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightRear");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        autoPan=new AutoPan(hardwares);
        autoPanCommand = new AutoPan.AutoPanCommand(
                autoPan,
                odo
        );

        CommandScheduler.getInstance().schedule(autoPanCommand);

        while (opModeIsActive()){

            if(gamepad1.dpad_up){
                sensors.odo.setPosition(new Pose2D(DistanceUnit.CM,0,-360.68, AngleUnit.DEGREES,0));
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
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
        }
    }
}
