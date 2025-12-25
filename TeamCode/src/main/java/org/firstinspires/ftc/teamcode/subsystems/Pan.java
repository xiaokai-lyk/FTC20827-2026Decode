package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

public class Pan {
    private final double panTicksPerDegree = 21.84;
    private final double panPower = 1.0;
    private final Limelight3A limelight;
    private final DcMotorEx panMotor;

    public Pan(@NonNull Hardwares hardwares) {
        this.limelight = hardwares.sensors.limelight;
        this.panMotor = hardwares.motors.pan;
    }

    public void setPanAngle(double angleDegrees) {
        int targetPos = (int) (normalizeAngle(angleDegrees) * panTicksPerDegree);
        panMotor.setPower(panPower);
        panMotor.setTargetPosition(targetPos);
    }

    public void hold() {
        panMotor.setPower(0);
    }

    private double normalizeAngle(double angleDegrees) {
        while (angleDegrees >= 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;
        return angleDegrees;
    }

    public double getCurrentAngle() {
        return panMotor.getCurrentPosition() / panTicksPerDegree;
    }

    public boolean isWithinLimits(double angleDeg) {
        return angleDeg >= -180 && angleDeg <= 180;
    }

    public static class AutoPanCommand extends CommandBase {
        private final Pan pan;
        private final OdoData odoData;
        private final double fieldWidth = 365.75;

        public AutoPanCommand(Pan pan, OdoData odoData) {
            this.pan = pan;
            this.odoData = odoData;
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            double robotHeading = odoData.getHeadingRadians();
            double robotX = odoData.getRobotX();
            double robotY = odoData.getRobotY();

            double panCurrAngle = pan.getCurrentAngle();
            double odoTargetAngle = (Math.toDegrees(Math.atan((fieldWidth / 2 + robotY) / (fieldWidth - robotX))) + robotHeading);

            LLResult result = pan.limelight.getLatestResult();
            double cameraTargetAngle = panCurrAngle + result.getTx();

            if (result.isValid() && pan.isWithinLimits(cameraTargetAngle)) {
                pan.setPanAngle(cameraTargetAngle);
            } else {
                pan.setPanAngle(odoTargetAngle);
            }
        }

        @Override
        public void end(boolean interrupted) { pan.hold(); }

        @Override
        public boolean isFinished() { return false; }
    }
}
