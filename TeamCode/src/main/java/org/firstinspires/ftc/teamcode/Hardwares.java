package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.XKHardware;

public class Hardwares {
    public Sensors sensors;
    public Motors motors;
    private HardwareMap hardwareMap;

    public class Sensors{
        public XKHardware<Limelight3A> mainCamera;
        public XKHardware<SparkFunOTOS> otos;

        public Sensors(@NonNull HardwareMap hardwareMap){
//            mainCamera = XKHardware.of(hardwareMap, "mainCamera", Limelight3A.class);
            otos = XKHardware.of(hardwareMap, "otos", SparkFunOTOS.class);
        }
    }

    public class Motors{
        public XKHardware<DcMotorEx> mFrontLeft, mFrontRight, mBackLeft, mBackRight;
        public Motors(@NonNull HardwareMap hardwareMap){
            mFrontLeft = XKHardware.of(hardwareMap, "frontLeft", DcMotorEx.class);
            mFrontRight = XKHardware.of(hardwareMap, "frontRight", DcMotorEx.class);
            mBackLeft = XKHardware.of(hardwareMap, "backLeft", DcMotorEx.class);
            mBackRight = XKHardware.of(hardwareMap, "backRight", DcMotorEx.class);
        }
    }

    public Hardwares(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        sensors = new Sensors(hardwareMap);
        motors = new Motors(hardwareMap);
    }
}
