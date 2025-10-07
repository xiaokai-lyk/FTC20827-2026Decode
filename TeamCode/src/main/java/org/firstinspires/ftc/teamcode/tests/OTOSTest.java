package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.XKHardware;

@TeleOp(name="OTOS Test", group="Tests")
public class OTOSTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Hardwares hardwares = new Hardwares(hardwareMap);
        XKHardware<SparkFunOTOS> otos = hardwares.sensors.otos;
        otos.device().setPosition(new SparkFunOTOS.Pose2D(0,0,0));
        otos.device().calibrateImu();
        while (isStarted()){
            telemetry.addData("x", otos.device().getPosition().x);
            telemetry.addData("y", otos.device().getPosition().y);
            telemetry.addData("heading", otos.device().getPosition().h);
            telemetry.update();
        }
    }
}
