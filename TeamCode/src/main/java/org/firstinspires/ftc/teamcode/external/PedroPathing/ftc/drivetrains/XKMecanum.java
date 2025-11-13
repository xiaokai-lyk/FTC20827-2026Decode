package org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class XKMecanum extends Mecanum{
    /**
     * This creates a new Mecanum, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     *
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param mecanumConstants this is the MecanumConstants object that contains the names of the motors and directions etc.
     */

    private double motorCachingThreshold;


    public XKMecanum(@NonNull HardwareMap hardwareMap, @NonNull MecanumConstants mecanumConstants) {
        super(hardwareMap, mecanumConstants);
    }

    public void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void updateConstants() {
        this.motorCachingThreshold = constants.motorCachingThreshold;

        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
    }

    @Override
    public void runDrive(double[] driveVelocities) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - driveVelocities[i]) > motorCachingThreshold) {
                motors.get(i).setVelocity(driveVelocities[i]);
            }
        }
    }
}
