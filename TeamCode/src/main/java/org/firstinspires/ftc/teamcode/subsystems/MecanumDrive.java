package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;

public class MecanumDrive {
    public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
    private DcMotor.RunMode currentMode = null;

    public MecanumDrive(@NonNull Hardwares hardwares){
        this.mFrontLeft = hardwares.motors.mFrontLeft;
        this.mFrontRight = hardwares.motors.mFrontRight;
        this.mBackLeft = hardwares.motors.mBackLeft;
        this.mBackRight = hardwares.motors.mBackRight;
    }

    public void runWithEncoders(){
        if (currentMode != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            mFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            currentMode = DcMotorEx.RunMode.RUN_USING_ENCODER;
        }
    }

    public void runWithoutEncoders(){
        if (currentMode != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            mFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            currentMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        }
    }

    public void setPower(double[] potentials){
        mFrontLeft.setPower(potentials[0]);
        mFrontRight.setPower(potentials[1]);
        mBackLeft.setPower(potentials[2]);
        mBackRight.setPower(potentials[3]);
    }

    public void setVelocity(double[] potentials){
        mFrontLeft.setVelocity(potentials[0] * Constants.driveMaxVelocity);
        mFrontRight.setVelocity(potentials[1] * Constants.driveMaxVelocity);
        mBackLeft.setVelocity(potentials[2] * Constants.driveMaxVelocity);
        mBackRight.setVelocity(potentials[3] * Constants.driveMaxVelocity);
    }

    public double[] calculateComponents(double x, double y, double rotate, double headingRadians, double speedCoefficient){
        double rotX = x * Math.cos(-headingRadians) - y * Math.sin(-headingRadians);
        double rotY = x * Math.sin(-headingRadians) + y * Math.cos(-headingRadians);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator * speedCoefficient;
        double backLeftPower = (rotY - rotX + rotate) / denominator * speedCoefficient;
        double frontRightPower = (rotY - rotX - rotate) / denominator * speedCoefficient;
        double backRightPower = (rotY + rotX - rotate) / denominator * speedCoefficient;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }
}
