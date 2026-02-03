package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Servo Test", group = "Tests")
public class ServoTest extends LinearOpMode {
    public static String servoName = "servo";
    public static double minAngle = 0;
    public static double maxAngle = 300;
    public static boolean servoInverted = false;
    public static double angleDeg = 0;

    @Override
    public void runOpMode() {
        ServoEx servo = new SimpleServo(hardwareMap, servoName, minAngle, maxAngle, AngleUnit.DEGREES);
        servo.setInverted(servoInverted);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            servo.turnToAngle(angleDeg);

            telemetry.addData("angle", angleDeg);
            telemetry.update();

            dashboard.getTelemetry().addData("angle", servo.getAngle());
            dashboard.getTelemetry().update();
        }
    }
}
