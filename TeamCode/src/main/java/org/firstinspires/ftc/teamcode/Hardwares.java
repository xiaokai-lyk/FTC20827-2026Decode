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
            odo.setOffsets(0, -16.8, DistanceUnit.CM); // needs to be calibrated
            odo.recalibrateIMU();
            odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

            limelight = getHardware(hardwareMap, "limelight", Limelight3A.class);
            limelight.pipelineSwitch(7);
            limelight.setPollRateHz(250);
            limelight.start();
        }
    }

    public static class Motors{
        public DcMotorEx mLeftFront, mRightFront, mLeftRear, mRightRear, preShooter, shooterFront, shooterBack, intake, pan;
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

//            shooterFront = getHardware(hardwareMap, "shooterFront", DcMotorEx.class);
//            shooterBack = getHardware(hardwareMap, "shooterBack", DcMotorEx.class);
            intake = getHardware(hardwareMap, "intake", DcMotorEx.class);
//            shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            shooterFront.setDirection(DcMotorEx.Direction.REVERSE);
//            shooterBack.setDirection(DcMotorEx.Direction.FORWARD);
            intake.setDirection(DcMotorEx.Direction.REVERSE);
//            shooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            shooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pan = getHardware(hardwareMap, "pan", DcMotorEx.class);
            pan.setTargetPosition(0);
            pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
    }

    public static class Servos{
        public ServoEx gate;
        public Servos(@NonNull HardwareMap hardwareMap){
//            gate = getHardware(hardwareMap, "gate", ServoEx.class);
        }
    }

    public Hardwares(HardwareMap hardwareMap){
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);
    }
}
