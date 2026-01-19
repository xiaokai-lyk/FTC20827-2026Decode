package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;
    public Servos servos;

    public static <T> T getHardware(@NonNull HardwareMap hardwareMap, String name, Class<T> clazz){
        return hardwareMap.get(clazz, name);
    }

    public static class Sensors{
        public GoBildaPinpointDriver odo;
        public Limelight3A limelight;

        public Sensors(@NonNull HardwareMap hardwareMap){
            odo = getHardware(hardwareMap, "pinpoint", GoBildaPinpointDriver.class);
            odo.setOffsets(-96, -72, DistanceUnit.MM);
            odo.recalibrateIMU();
            odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

            limelight = getHardware(hardwareMap, "limelight", Limelight3A.class);
            limelight.pipelineSwitch(7);
            limelight.setPollRateHz(250);
            limelight.start();
        }
    }

    public static class Motors{
        public DcMotorEx mLeftFront, mRightFront, mLeftRear, mRightRear, preShooter, shooterLeft, shooterRight, intake, pan;
        public Motors(@NonNull HardwareMap hardwareMap){
            mLeftFront = getHardware(hardwareMap, "leftFront", DcMotorEx.class);
            mRightFront = getHardware(hardwareMap, "rightFront", DcMotorEx.class);
            mLeftRear = getHardware(hardwareMap, "leftRear", DcMotorEx.class);
            mRightRear = getHardware(hardwareMap, "rightRear", DcMotorEx.class);

            mLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
            mRightFront.setDirection(DcMotorEx.Direction.REVERSE);
            mLeftRear.setDirection(DcMotorEx.Direction.REVERSE);
            mRightRear.setDirection(DcMotorEx.Direction.REVERSE);

            mLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake = getHardware(hardwareMap, "intake", DcMotorEx.class);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorEx.Direction.REVERSE);

            shooterLeft = getHardware(hardwareMap, "shooterLeft", DcMotorEx.class);
            shooterRight = getHardware(hardwareMap, "shooterRight", DcMotorEx.class);
            shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
            shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
            shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pan = getHardware(hardwareMap, "pan", DcMotorEx.class);
        }
    }

    public static class Servos{
        public ServoEx gate, pitch;
        public Servos(@NonNull HardwareMap hardwareMap){
//            gate = getHardware(hardwareMap, "gate", ServoEx.class);
//            pitch = getHardware(hardwareMap, "pitch", ServoEx.class);
        }
    }

    public Hardwares(HardwareMap hardwareMap){
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);
    }
}
