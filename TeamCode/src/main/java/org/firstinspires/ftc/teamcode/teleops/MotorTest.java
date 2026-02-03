package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Motor Test", group = "Tests")
public class MotorTest extends LinearOpMode {
    public static String motorName = "motor";
    public static boolean motorReverse = false;
    public static double motorPower = 1;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorReverse) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else motor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(motorPower);

            telemetry.addData("motor velocity", motor.getVelocity());
            telemetry.update();

            dashboard.getTelemetry().addData("motor velocity", motor.getVelocity());
            dashboard.getTelemetry().update();
        }
    }
}
