package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class Drive {
    private static final double MAX_VELOCITY = 2900.0;
    private DcMotorEx mLeftFront, mRightFront, mLeftRear, mRightRear;
    
    // 机器人运动学补偿因子
    private static final double TRANSLATION_COMPENSATION_FACTOR = 1.1;

    public Drive(@NonNull Hardwares hardwares){
        this.mLeftFront = hardwares.motors.mLeftFront;
        this.mRightFront = hardwares.motors.mRightFront;
        this.mLeftRear = hardwares.motors.mLeftRear;
        this.mRightRear = hardwares.motors.mRightRear;
    }

    public void setPower(@NonNull double[] potentials){
        mLeftFront.setPower(potentials[0]);
        mRightFront.setPower(potentials[1]);
        mLeftRear.setPower(potentials[2]);
        mRightRear.setPower(potentials[3]);
    }

    public void setVelocity(@NonNull double[] potentials){
        mLeftFront.setVelocity(potentials[0] * MAX_VELOCITY);
        mRightFront.setVelocity(potentials[1] * MAX_VELOCITY);
        mLeftRear.setVelocity(potentials[2] * MAX_VELOCITY);
        mRightRear.setVelocity(potentials[3] * MAX_VELOCITY);
    }

    public double[] calculateComponents(double x, double y, double rotate, double headingRadians, double speedCoefficient){
        double rotX = x * Math.cos(-headingRadians) - y * Math.sin(-headingRadians);
        double rotY = x * Math.sin(-headingRadians) + y * Math.cos(-headingRadians);
        rotX = rotX * TRANSLATION_COMPENSATION_FACTOR;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator * speedCoefficient;
        double backLeftPower = (rotY - rotX + rotate) / denominator * speedCoefficient;
        double frontRightPower = (rotY - rotX - rotate) / denominator * speedCoefficient;
        double backRightPower = (rotY + rotX - rotate) / denominator * speedCoefficient;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }


    public static class DriveCommand extends CommandBase {
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier rotateSupplier;
        private final DoubleSupplier ySupplier;
        private final double speedCoefficient;
        private final Drive drive;
        private final Supplier<OdoData> odometerDataSupplier;
        private final boolean useEncoders;
        private final boolean fieldCentric;

        public double dampedX, dampedY, dampedRotate;

        public DriveCommand(
                Drive drive,
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                DoubleSupplier rotateSupplier,
                Supplier<OdoData> odometerData,
                double speedCoefficient,
                boolean useEncoders,
                boolean fieldCentric) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.rotateSupplier = rotateSupplier;
            this.odometerDataSupplier = odometerData;
            this.speedCoefficient = speedCoefficient;
            this.useEncoders = useEncoders;
            this.fieldCentric = fieldCentric;
        }

        @Override
        public void execute() {
            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double rotate = rotateSupplier.getAsDouble();
            OdoData odometerData = odometerDataSupplier.get();
            
            double currentHeading;
            if(fieldCentric)
            {
                currentHeading = odometerData.getHeadingRadians();
            }else{
                currentHeading = 0.0;
            }

            double[] potentials = drive.calculateComponents(
                    x,
                    y,
                    -rotate,
                    currentHeading,
                    speedCoefficient
            );

            if (useEncoders) {
                drive.setVelocity(potentials);
            } else {
                drive.setPower(potentials);
            }
        }
    }
}