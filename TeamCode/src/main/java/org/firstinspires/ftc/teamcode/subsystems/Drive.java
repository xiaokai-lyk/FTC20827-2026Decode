package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.AngularDamping;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.TranslationalDamping;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive {
    public DcMotorEx mFrontLeft, mFrontRight, mBackLeft, mBackRight;
    private DcMotor.RunMode currentMode = null;

    public Drive(@NonNull Hardwares hardwares){
        this.mFrontLeft = hardwares.motors.mFrontLeft;
        this.mFrontRight = hardwares.motors.mFrontRight;
        this.mBackLeft = hardwares.motors.mBackLeft;
        this.mBackRight = hardwares.motors.mBackRight;
    }

    public void runWithEncoders(){
        if (currentMode != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            mFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            currentMode = DcMotorEx.RunMode.RUN_USING_ENCODER;
        }
    }

    public void runWithoutEncoders(){
        if (currentMode != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            mFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            mBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            currentMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        }
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

    public void runToPosition(double x, double y, @NonNull OdometerData data) {
        double[] potentials = this.calculateComponents(
            x-data.getRobotPosition().getX(DistanceUnit.METER),
            y-data.getRobotPosition().getY(DistanceUnit.METER),
            0,
            data.getHeadingRadians(),
            1.0
        );
        this.setVelocity(potentials);
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

        public double dampedX, dampedY, dampedRotate;

        public DriveCommand(
                Drive drive,
                DoubleSupplier x,
                DoubleSupplier y,
                DoubleSupplier rotate,
                java.util.function.Supplier<OdometerData> odometerDataSupplier,
                DoubleSupplier speedCoefficient,
                BooleanSupplier useEncoders
        ) {
            this.drive = drive;
            this.x = x;
            this.rotate = rotate;
            this.y = y;
            this.speedCoefficient = speedCoefficient;
            this.odometerDataSupplier = odometerDataSupplier;
            this.useEncoders = useEncoders;
        }

        @Override
        public void execute() {
            double rotateInput = rotate.getAsDouble();
            OdometerData data = odometerDataSupplier.get();
            double currentHeading = data.getHeadingRadians();

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
                drive.runWithEncoders();
                drive.setVelocity(potentials);
            } else {
                drive.runWithoutEncoders();
                drive.setPower(potentials);
            }
        }
    }
}
