package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;

    public static <T> T getHardware(@NonNull HardwareMap hardwareMap, String name, Class<T> clazz){
        return hardwareMap.get(clazz, name);
    }

    public static class Sensors{
        public GoBildaPinpointDriver odo;

        public Sensors(@NonNull HardwareMap hardwareMap){
            odo = getHardware(hardwareMap, "odo", GoBildaPinpointDriver.class);
            odo.setOffsets(100,48, DistanceUnit.MM);
            odo.recalibrateIMU();
            odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
        }
    }

    public static class Motors{
        public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight, preShooter, shooterFront, shooterBack, intake;
        public Motors(@NonNull HardwareMap hardwareMap){
            mFrontLeft = getHardware(hardwareMap, "frontLeft", DcMotorEx.class);
            mFrontRight = getHardware(hardwareMap, "frontRight", DcMotorEx.class);
            mBackLeft = getHardware(hardwareMap, "backLeft", DcMotorEx.class);
            mBackRight = getHardware(hardwareMap, "backRight", DcMotorEx.class);
            mFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            mFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
            mBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
            mBackRight.setDirection(DcMotorEx.Direction.FORWARD);
            mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            preShooter = getHardware(hardwareMap, "preShooter", DcMotorEx.class);
            shooterFront = getHardware(hardwareMap, "shooterFront", DcMotorEx.class);
            shooterBack = getHardware(hardwareMap, "shooterBack", DcMotorEx.class);
            intake = getHardware(hardwareMap, "intake", DcMotorEx.class);
            preShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            shooterFront.setDirection(DcMotorEx.Direction.REVERSE);
            shooterBack.setDirection(DcMotorEx.Direction.FORWARD);
            preShooter.setDirection(DcMotorEx.Direction.REVERSE);
            intake.setDirection(DcMotorEx.Direction.REVERSE);
            shooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public Hardwares(HardwareMap hardwareMap){
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
    }
}
