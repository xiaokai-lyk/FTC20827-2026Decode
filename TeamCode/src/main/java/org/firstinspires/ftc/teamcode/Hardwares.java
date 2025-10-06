package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;
    private HardwareMap hardwareMap;

    private <T> T getHardware(String name, Class<T> type) {
        return this.hardwareMap.get(type, name);
    }


    public class Sensors{
        public Limelight3A mainCamera;
        public SparkFunOTOS otos;

        public Sensors(@NonNull HardwareMap hardwareMap){
//            mainCamera = getHardware(hardwareMap, "mainCamera", Limelight3A.class);
            otos = getHardware("otos", SparkFunOTOS.class);
        }
    }

    public class Motors{
        public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
        public Motors(@NonNull HardwareMap hardwareMap){
            mFrontLeft = getHardware("frontLeft", DcMotorEx.class);
            mFrontRight = getHardware("frontRight", DcMotorEx.class);
            mBackLeft = getHardware("backLeft", DcMotorEx.class);
            mBackRight = getHardware("backRight", DcMotorEx.class);
        }
    }




    public Hardwares(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
    }
}
