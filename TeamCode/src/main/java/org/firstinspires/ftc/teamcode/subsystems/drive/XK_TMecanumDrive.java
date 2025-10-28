package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardwares;

public class XK_TMecanumDrive {
    // Motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // IMU
    private BHI260IMU imu;
    private Telemetry telemetry;

    private Hardwares hardwares;

    // Wheel configuration (tunables)
    public double maxPower = 1.0;

    // Constructor
    public XK_TMecanumDrive(Hardwares hardwares, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwares = hardwares;
    }

    // Constructor with max power
    public XK_TMecanumDrive(Hardwares hardwares, Telemetry telemetry, double maxPower) {
        this.telemetry = telemetry;
        this.hardwares = hardwares;
        this.maxPower = maxPower;
    }

    // Initialize hardware - pass the names used in robot configuration
    public void init() {
        // Map motors
        frontLeft  = hardwares.motors.mFrontLeft;
        frontRight = hardwares.motors.mFrontRight;
        backLeft   = hardwares.motors.mBackLeft;
        backRight  = hardwares.motors.mBackRight;

        // Motor directions - change if your robot is mirrored
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Default modes and brake behavior
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwares.sensors.insideIMU;
    }

    // Robot-centric mecanum drive: x = strafe (-1..1), y = forward (-1..1), rot = rotation (-1..1)
    public void driveCartesian(double x, double y, double rot) {
        // Combine inputs: note y is typically inverted depending on joystick mapping
        double fl = y + x + rot;
        double fr = y - x - rot;
        double bl = y - x + rot;
        double br = y + x - rot;

        // Normalize to maxPower
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        setMotorPowers(fl * maxPower, fr * maxPower, bl * maxPower, br * maxPower);
    }

    // Field-centric drive: rotates input by negative heading so driver inputs are field-relative
    public void driveFieldCentric(double x, double y, double rot) {
        double heading = getHeadingRadians();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        // rotate joystick vector
        double xPrime = x * cos - y * sin;
        double yPrime = x * sin + y * cos;

        driveCartesian(xPrime, yPrime, rot);
    }

    // Set individual motor powers (range will be clipped)
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(Range.clip(fl, -maxPower, maxPower));
        frontRight.setPower(Range.clip(fr, -maxPower, maxPower));
        backLeft.setPower(Range.clip(bl, -maxPower, maxPower));
        backRight.setPower(Range.clip(br, -maxPower, maxPower));
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    // Simple helpers
    public double getHeadingDegrees() {
        if (imu == null) return 0.0;
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    public double getHeadingRadians() {
        return Math.toRadians(getHeadingDegrees());
    }

    // Return encoder positions (ticks) for each wheel
    public long[] getWheelPositions() {
        return new long[] {
            frontLeft.getCurrentPosition(),
            frontRight.getCurrentPosition(),
            backLeft.getCurrentPosition(),
            backRight.getCurrentPosition()
        };
    }

    // Telemetry helper
    public void addDataToTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("FL power/enc", "%4.2f / %d", frontLeft.getPower(), frontLeft.getCurrentPosition());
        telemetry.addData("FR power/enc", "%4.2f / %d", frontRight.getPower(), frontRight.getCurrentPosition());
        telemetry.addData("BL power/enc", "%4.2f / %d", backLeft.getPower(), backLeft.getCurrentPosition());
        telemetry.addData("BR power/enc", "%4.2f / %d", backRight.getPower(), backRight.getCurrentPosition());
        telemetry.addData("Heading", "%4.1f deg", getHeadingDegrees());
    }
}
