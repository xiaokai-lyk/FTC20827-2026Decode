package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;

    public static class Sensors{
        public Limelight3A mainCamera;
        public Sensors(@NonNull HardwareMap hardwareMap){
            mainCamera = hardwareMap.get(Limelight3A.class, "mainCamera");
        }
    }

    public static class Motors{
        public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
        public Motors(@NonNull HardwareMap hardwareMap){
            mFrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            mFrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            mBackLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            mBackRight = hardwareMap.get(DcMotorEx.class, "backRight");
        }
    }


    public Hardwares(HardwareMap hardwareMap){
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
    }
}
