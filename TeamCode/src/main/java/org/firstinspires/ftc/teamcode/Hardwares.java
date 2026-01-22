package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.jetbrains.annotations.Contract;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;
    public Servos servos;

    public static <T> T getHardware(@NonNull HardwareMap hardwareMap, String name, Class<T> clazz){
        return hardwareMap.get(clazz, name);
    }

    @NonNull
    @Contract("_, _, _, _ -> new")
    public static ServoEx getHardware(@NonNull HardwareMap hardwareMap, String name, double minAngleDegree, double maxAngleDegree){
        return new SimpleServo(hardwareMap, name, minAngleDegree, maxAngleDegree);
    }

    public static class Sensors{
        public GoBildaPinpointDriver odo;

        public Sensors(@NonNull HardwareMap hardwareMap){
            odo = getHardware(hardwareMap, "pinpoint", GoBildaPinpointDriver.class);
            odo.setOffsets(-96, -72, DistanceUnit.MM);
            odo.recalibrateIMU();
            odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }
    }

    public static class Motors{
        public DcMotorEx mLeftFront, mRightFront, mLeftRear, mRightRear, preShooter, shooterLeft, shooterRight, intake, pan;
        public Motors(@NonNull HardwareMap hardwareMap){
            mLeftFront = getHardware(hardwareMap, "leftFront", DcMotorEx.class);
            mRightFront = getHardware(hardwareMap, "rightFront", DcMotorEx.class);
            mLeftRear = getHardware(hardwareMap, "leftRear", DcMotorEx.class);
            mRightRear = getHardware(hardwareMap, "rightRear", DcMotorEx.class);

            intake = getHardware(hardwareMap, "intake", DcMotorEx.class);

            shooterLeft = getHardware(hardwareMap, "shooterLeft", DcMotorEx.class);
            shooterRight = getHardware(hardwareMap, "shooterRight", DcMotorEx.class);

            pan = getHardware(hardwareMap, "pan", DcMotorEx.class);
        }
    }

    public static class Servos{
        public ServoEx gate, pitch;
        public Servos(@NonNull HardwareMap hardwareMap){
            gate = getHardware(hardwareMap, "gate", 0, 360);
            pitch = getHardware(hardwareMap, "pitch", 0, 300);
        }
    }

    public Hardwares(HardwareMap hardwareMap){
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);
    }
}
