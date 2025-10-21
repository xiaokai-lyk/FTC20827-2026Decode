package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HelloWorldLinear extends LinearOpMode {
    @Override
    public void runOpMode() {
        while (opModeIsActive()) {
            telemetry.addData("Hello", "World");
            telemetry.update();
        }
    }
}
