package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.AngularDamping;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.TranslationalDamping;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Drive {
    public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
    private DcMotor.RunMode currentMode = null;

    public Drive(@NonNull Hardwares hardwares){
        this.mFrontLeft = hardwares.motors.mFrontLeft;
        this.mFrontRight = hardwares.motors.mFrontRight;
        this.mBackLeft = hardwares.motors.mBackLeft;
        this.mBackRight = hardwares.motors.mBackRight;
    }

    public void setPower(@NonNull double[] potentials){
        mFrontLeft.setPower(potentials[0]);
        mFrontRight.setPower(potentials[1]);
        mBackLeft.setPower(potentials[2]);
        mBackRight.setPower(potentials[3]);
    }

    public void setVelocity(@NonNull double[] potentials){
        mFrontLeft.setVelocity(potentials[0] * Constants.driveMaxVelocity);
        mFrontRight.setVelocity(potentials[1] * Constants.driveMaxVelocity);
        mBackLeft.setVelocity(potentials[2] * Constants.driveMaxVelocity);
        mBackRight.setVelocity(potentials[3] * Constants.driveMaxVelocity);
    }

    public double[] calculateComponents(double x, double y, double rotate, double headingRadians, double speedCoefficient){
        double rotX = x * Math.cos(-headingRadians) - y * Math.sin(-headingRadians);
        double rotY = x * Math.sin(-headingRadians) + y * Math.cos(-headingRadians);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator * speedCoefficient;
        double backLeftPower = (rotY - rotX + rotate) / denominator * speedCoefficient;
        double frontRightPower = (rotY - rotX - rotate) / denominator * speedCoefficient;
        double backRightPower = (rotY + rotX - rotate) / denominator * speedCoefficient;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }


    public static class DriveCommand extends CommandBase {
        private final DoubleSupplier x;
        private final DoubleSupplier rotate;
        private final DoubleSupplier y;
        private final DoubleSupplier speedCoefficient;
        private final Drive drive;
        private final java.util.function.Supplier<OdometerData> odometerDataSupplier;
        private final BooleanSupplier useEncoders;
        // 平移阻尼实例
        private final TranslationalDamping translationalDamping = new TranslationalDamping();
        private final AngularDamping angularDamping = new AngularDamping();
        private final BooleanSupplier fieldCentric;

        public double dampedX, dampedY, dampedRotate;

        public DriveCommand(
            Drive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotate,
            Supplier<OdometerData> odometerDataSupplier,
            DoubleSupplier speedCoefficient,
            BooleanSupplier useEncoders, BooleanSupplier fieldCentric) {
            this.drive = drive;
            this.x = x;
            this.rotate = rotate;
            this.y = y;
            this.speedCoefficient = speedCoefficient;
            this.odometerDataSupplier = odometerDataSupplier;
            this.useEncoders = useEncoders;
            this.fieldCentric = fieldCentric;
        }

        @Override
        public void execute() {
            double rotateInput = rotate.getAsDouble();
            OdometerData data = odometerDataSupplier.get();
            double currentHeading;
            if(fieldCentric.getAsBoolean())
            {
                currentHeading = data.getHeadingRadians();
            }else{
                currentHeading = 0.0;
            }

            AngularDamping.Result angResult = angularDamping.updateAndApply(
                    rotateInput,
                    currentHeading,
                    data,
                    x.getAsDouble(),
                    y.getAsDouble()
            );
            dampedRotate = angResult.rotateCmd;
            double yawRateFiltered = angResult.yawRateFiltered; // 传给平移阻尼耦合因子
            TranslationalDamping.Result dampingResult = translationalDamping.updateAndApply(
                    x.getAsDouble(),
                    y.getAsDouble(),
                    yawRateFiltered,
                    data
            );
            dampedX = dampingResult.x;
            dampedY = dampingResult.y;
            double[] potentials = drive.calculateComponents(
                    dampedX,
                    dampedY,
                    -dampedRotate,
                    currentHeading,
                    speedCoefficient.getAsDouble()
            );
            if (useEncoders.getAsBoolean()) {
                drive.setVelocity(potentials);
            } else {
                drive.setPower(potentials);
            }
        }
    }
}
