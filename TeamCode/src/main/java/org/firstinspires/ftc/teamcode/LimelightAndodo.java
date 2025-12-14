package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Limelight&odo", group = "TeleOp")
public class LimelightAndodo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(7);
        limelight.start();

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx pan = hardwareMap.get(DcMotorEx.class, "pan");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize pan encoder
        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Use RUN_USING_ENCODER for velocity control via setVelocity
        pan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tunable parameters (adjust on your robot)
        final double TICKS_PER_DEGREE = 72.1;      // encoder ticks per 1 degree pan rotation (tune this)
        final double TX_DEGREE_THRESHOLD = 1.0;    // only command if |tx| > threshold (deg)
        // Mechanical limits: set wide defaults (change to your robot's limits)
        final int MIN_PAN_TICKS = -10000;           // mechanical minimum encoder ticks (tune)
        final int MAX_PAN_TICKS = 10000;            // mechanical maximum encoder ticks (tune)

        // PID tuning (units: degrees -> degrees/sec output)
        final double KP = 40.0;    // proportional gain
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
            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((x + y + rx) / denominator);
            backLeft.setPower((x - y + rx) / denominator);
            frontRight.setPower((x - y - rx) / denominator);
            backRight.setPower((x + y - rx) / denominator);

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
                double velCounter = -gamepad1.right_stick_x * 45 * TICKS_PER_DEGREE;
                double txDeg = res.getTx(); // positive = target to right
                telemetry.addData("tx(deg)", String.format("%.2f", txDeg));

                if (Math.abs(txDeg) > TX_DEGREE_THRESHOLD) {
                    // Use FTCLib PIDFController: measurement is txDeg, setpoint is 0
                    double velCmd = pidfController.calculate(-txDeg * TICKS_PER_DEGREE);

                    // Clamp velocity (deg/sec)
                    if (velCmd > MAX_VELOCITY_DPS) velCmd = MAX_VELOCITY_DPS;
                    if (velCmd < -MAX_VELOCITY_DPS) velCmd = -MAX_VELOCITY_DPS;

                    // Safety: if moving would exceed mechanical limits, reduce/zero velocity
                    int current = pan.getCurrentPosition();
                    if ((current <= MIN_PAN_TICKS && velCmd < 0) || (current >= MAX_PAN_TICKS && velCmd > 0)) {
                        velCmd = 0.0;
                    }

                    // send velocity command (ticks per second)
                    pan.setVelocity(velCmd + velCounter);

                    telemetry.addData("VelCmd(dps)", String.format("%.2f", velCmd));

                } else {
                    // within deadband -> stop velocity
                    pan.setVelocity(0);
                    pidfController.reset();
                }

            } else {
                // no valid target or auto-pan disabled -> stop motor (unless operator is commanding manual pan with X)
                if (!gamepad1.x) {
                    pan.setVelocity(0);
                    continue;
                }
                // turn to zero position when target is lost
                double turningVelocity=0;
                if (pan.getCurrentPosition()>0){
                    turningVelocity =-250;
                }
                else if (pan.getCurrentPosition()<0){
                    turningVelocity=250;
                }
                if(pan.getCurrentPosition()>45||pan.getCurrentPosition()<-45){
                    pan.setVelocity(turningVelocity);
                }
                else{
                    pan.setVelocity(0);
                }

                telemetry.addData("AutoPan", autoPanEnabled ? "enabled (no valid target)" : "disabled");
            }


            telemetry.addData("PanPos", pan.getCurrentPosition());
            telemetry.addData("PanBusy", pan.isBusy());
            telemetry.addData("TargetPos", pan.getTargetPosition());
            telemetry.addData("RunMode", pan.getMode().toString());
            telemetry.addData("TicksPerDegree", String.format("%.2f", TICKS_PER_DEGREE));
            telemetry.addData("x",gamepad1.x);
            telemetry.addData("y",gamepad1.y);
            telemetry.update();
        }
    }
}
