package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardwares;

@Disabled
@TeleOp(name="OTOS Test", group="Tests")
public class OTOSTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Hardwares hardwares = new Hardwares(hardwareMap);
        SparkFunOTOS otos = hardwares.sensors.otos;
        otos.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
        otos.calibrateImu();
        while (isStarted()){
            telemetry.addData("x", otos.getPosition().x);
            telemetry.addData("y", otos.getPosition().y);
            telemetry.addData("heading", otos.getPosition().h);
            telemetry.update();
        }
    }
}
