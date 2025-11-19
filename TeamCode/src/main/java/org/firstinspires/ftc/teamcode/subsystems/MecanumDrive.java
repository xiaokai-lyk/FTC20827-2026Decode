package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

public class MecanumDrive {
    public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;

    public MecanumDrive(@NonNull Hardwares hardwares){
        this.mFrontLeft = hardwares.motors.mFrontLeft;
        this.mFrontRight = hardwares.motors.mFrontRight;
        this.mBackLeft = hardwares.motors.mBackLeft;
        this.mBackRight = hardwares.motors.mBackRight;
    }

    public void driveFieldCentric(double x, double y, double rotate, double headingRadians, double powerCoefficient){
        double rotX = x * Math.cos(-headingRadians) - y * Math.sin(-headingRadians);
        double rotY = x * Math.sin(-headingRadians) + y * Math.cos(-headingRadians);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator * powerCoefficient;
        double backLeftPower = (rotY - rotX + rotate) / denominator * powerCoefficient;
        double frontRightPower = (rotY - rotX - rotate) / denominator * powerCoefficient;
        double backRightPower = (rotY + rotX - rotate) / denominator * powerCoefficient;


        mFrontLeft.setPower(frontLeftPower);
        mFrontRight.setPower(frontRightPower);
        mBackLeft.setPower(backLeftPower);
        mBackRight.setPower(backRightPower);
    }
}
