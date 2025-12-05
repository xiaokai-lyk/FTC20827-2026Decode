package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Pan PIDF Test", group = "Tests")
public class PanPIDFTest extends LinearOpMode {
    public static double KP = 4.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double TICKS_PER_DEGREE = 72.1;
    public static double degrees = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx pan = hardwareMap.get(DcMotorEx.class, "pan");

        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        double txDeg;

        waitForStart();

        while (opModeIsActive()) {
            PIDFController pidfController = new PIDFController(KP, KI, KD, 0);

            if (gamepad1.dpad_right) {
                txDeg = -degrees;
            } else if (gamepad1.dpad_left) {
                txDeg = degrees;
            } else {
                txDeg = 0.0;
            }

            double vel = pidfController.calculate(txDeg) * TICKS_PER_DEGREE;

            pan.setVelocity(vel);

            TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
            packet.put("pan/velocity", pan.getVelocity());
            packet.put("pan/target", vel);
            dashboard.sendTelemetryPacket(packet);

            dashboard.getTelemetry().update();
        }
    }
}
