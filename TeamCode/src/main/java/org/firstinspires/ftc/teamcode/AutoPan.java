package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "AutoPan", group = "TeleOp")
public class AutoPan extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(120);
        limelight.pipelineSwitch(7);
        limelight.start();

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx pan = hardwareMap.get(DcMotorEx.class, "pan");

        // Initialize pan encoder
        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Use RUN_USING_ENCODER for velocity control via setVelocity
        pan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tunable parameters (adjust on your robot)
        final double TICKS_PER_DEGREE = 0.33333;      // encoder ticks per 1 degree pan rotation (tune this)
        final double TX_DEGREE_THRESHOLD = 0.5;    // only command if |tx| > threshold (deg)
        // Mechanical limits: set wide defaults (change to your robot's limits)
        final int MIN_PAN_TICKS = -10000;           // mechanical minimum encoder ticks (tune)
        final int MAX_PAN_TICKS = 10000;            // mechanical maximum encoder ticks (tune)

        // PID tuning (units: degrees -> degrees/sec output)
        final double KP = 4.0;    // proportional gain
        final double KI = 0.02;   // integral gain
        final double KD = 0.8;    // derivative gain

        final PIDFController pidfController = new PIDFController(KP, KI, KD, 0);
        // we want the controller to drive tx -> 0
        pidfController.setSetPoint(0.0);

        final double MAX_VELOCITY_DPS = 300.0; // max commanded velocity in degrees/sec - tune safely

        boolean autoPanEnabled = true;            // start with auto-pan enabled
        long lastCommandTime = System.currentTimeMillis();
        long lastToggleTime = 0;

        telemetry.addData(">", "Ready - A toggle auto-pan, B recenter");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Drive controls (kept from original)
            double x = gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            frontLeft.setPower(x + y + rx);
            frontRight.setPower(x - y - rx);
            backLeft.setPower(x - y + rx);
            backRight.setPower(x + y - rx);

            // Toggle auto-pan with A (debounced)
            if (gamepad1.a && (System.currentTimeMillis() - lastToggleTime) > 300) {
                autoPanEnabled = !autoPanEnabled;
                lastToggleTime = System.currentTimeMillis();
            }

            // Re-center pan with B (reset encoder to 0)
            if (gamepad1.b && (System.currentTimeMillis() - lastToggleTime) > 300) {
                // reset encoder reference — keep motor in RUN_USING_ENCODER
                pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pidfController.reset();
                lastToggleTime = System.currentTimeMillis();
            }

            // Get Limelight data (may be null)
            LLResult res = limelight.getLatestResult();
            if (res != null) {
                telemetry.addData("tx", res.getTx());
                telemetry.addData("ty", res.getTy());
                telemetry.addData("ta", res.getTa());
                telemetry.addData("Latency", res.getCaptureLatency());
            } else {
                telemetry.addData("Limelight", "no result (null)");
            }

            // Auto-pan logic: use PIDFController.calculate() with degrees units -> produce deg/sec
            long now = System.currentTimeMillis();

            if (autoPanEnabled && res != null && res.isValid()) {
                double txDeg = res.getTx(); // positive = target to right
                telemetry.addData("tx(deg)", String.format("%.2f", txDeg));

                if (Math.abs(txDeg) > TX_DEGREE_THRESHOLD) {
                    // Use FTCLib PIDFController: measurement is txDeg, setpoint is 0
                    double velCmdDps = pidfController.calculate(txDeg);

                    // Clamp velocity (deg/sec)
                    if (velCmdDps > MAX_VELOCITY_DPS) velCmdDps = MAX_VELOCITY_DPS;
                    if (velCmdDps < -MAX_VELOCITY_DPS) velCmdDps = -MAX_VELOCITY_DPS;

                    // Convert deg/sec -> ticks/sec for setVelocity
                    double velCmdTps = velCmdDps * TICKS_PER_DEGREE;

                    // Safety: if moving would exceed mechanical limits, reduce/zero velocity
                    int current = pan.getCurrentPosition();
                    if ((current <= MIN_PAN_TICKS && velCmdTps < 0) || (current >= MAX_PAN_TICKS && velCmdTps > 0)) {
                        velCmdTps = 0.0;
                        velCmdDps = 0.0;
                    }

                    // send velocity command (ticks per second)
                    pan.setVelocity(velCmdTps);

                    telemetry.addData("VelCmd(dps)", String.format("%.2f", velCmdDps));
                    telemetry.addData("VelCmd(tps)", String.format("%.1f", velCmdTps));

                } else {
                    // within deadband -> stop velocity
                    pan.setVelocity(0);
                    pidfController.reset();
                }

            } else {
                // no valid target or auto-pan disabled -> stop motor (unless operator is commanding manual pan with X)
                if (!gamepad1.x) {
                    pan.setVelocity(0);
                } else {
                    // allow manual control with right stick x mapped to deg/sec
                    double manual = -gamepad1.right_stick_x; // adjust sign if needed
                    double manualVelDps = manual * MAX_VELOCITY_DPS;
                    double manualVelTps = manualVelDps * TICKS_PER_DEGREE;
                    pan.setVelocity(manualVelTps);
                    telemetry.addData("ManualVel(dps)", String.format("%.1f", manualVelDps));
                }
                telemetry.addData("AutoPan", autoPanEnabled ? "enabled (no valid target)" : "disabled");
            }


            telemetry.addData("PanPos", pan.getCurrentPosition());
            telemetry.addData("PanBusy", pan.isBusy());
            telemetry.addData("TargetPos", pan.getTargetPosition());
            telemetry.addData("RunMode", pan.getMode().toString());
            telemetry.addData("TicksPerDegree", String.format("%.2f", TICKS_PER_DEGREE));
            telemetry.update();
        }
    }
}
