package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class LimelightTeleOp extends LinearOpMode {
    final double TICKS_PER_DEGREE = 6.0;
    final double MAX_PAN_TICKS = 180 * TICKS_PER_DEGREE;
    final double MIN_PAN_TICKS = -180 * TICKS_PER_DEGREE;
    final double FIELD_WIDTH = 365.75;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(7);
        limelight.start();

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx pan = hardwareMap.get(DcMotorEx.class, "pan");

        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pan.setTargetPosition(0);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pan.setPower(0.6);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-2, 6, DistanceUnit.CM);
        odo.setPosition(new Pose2D(DistanceUnit.CM, FIELD_WIDTH / 6, 0, AngleUnit.DEGREES, 0));

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            double cenX = odo.getPosX(DistanceUnit.CM);
            double cenY = odo.getPosY(DistanceUnit.CM);
            double heading = odo.getHeading(AngleUnit.DEGREES);

            telemetry.addData("CenX", cenX);
            telemetry.addData("CenY", cenY);
            telemetry.addData("Heading", heading);

            // Drive controls (kept from original)
            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((x + y + rx) / denominator);
            backLeft.setPower((x - y + rx) / denominator);
            frontRight.setPower((x - y - rx) / denominator);
            backRight.setPower((x + y - rx) / denominator);

            LLResult res = limelight.getLatestResult();

            if (res.isValid()) {
                telemetry.addData("tx", res.getTx());
                telemetry.addData("ty", res.getTy());
                telemetry.addData("ta", res.getTa());
                telemetry.addData("Latency", res.getCaptureLatency());
            } else {
                telemetry.addData("Limelight", "no result (null)");
            }

            int panPos = pan.getCurrentPosition();
            if (res.isValid() && panPos > MIN_PAN_TICKS && panPos < MAX_PAN_TICKS) {
                pan.setTargetPosition(panPos + (int) (res.getTx() * TICKS_PER_DEGREE));
            } else {
                double target = (Math.toDegrees(Math.atan((FIELD_WIDTH / 2 + cenY) / (FIELD_WIDTH - cenX))) + heading) * TICKS_PER_DEGREE;
                pan.setTargetPosition((int) target);
            }

            telemetry.addData("PanPos", panPos);
            telemetry.update();
        }
    }
}
