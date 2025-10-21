package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;

    public static <T> T getHardware(HardwareMap hardwareMap, String name, Class<T> clazz){
        return hardwareMap.get(clazz, name);
    }

    public static class Sensors{
        @SuppressWarnings("unused")
        public Limelight3A mainCamera;
        public SparkFunOTOS otos;
        public BHI260IMU insideIMU;

        public Sensors(@NonNull HardwareMap hardwareMap){
//            mainCamera = getHardware(hardwareMap, "mainCamera", Limelight3A.class);
//            otos = getHardware(hardwareMap, "otos", SparkFunOTOS.class);
            insideIMU = getHardware(hardwareMap,"imu", BHI260IMU.class);
            BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            insideIMU.initialize(parameters);
        }
    }

    public static class Motors{
        public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
        public Motors(@NonNull HardwareMap hardwareMap){
            mFrontLeft = getHardware(hardwareMap, "frontLeft", DcMotorEx.class);
            mFrontRight = getHardware(hardwareMap, "frontRight", DcMotorEx.class);
            mBackLeft = getHardware(hardwareMap, "backLeft", DcMotorEx.class);
            mBackRight = getHardware(hardwareMap, "backRight", DcMotorEx.class);
            mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public Hardwares(HardwareMap hardwareMap){
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
    }
}
