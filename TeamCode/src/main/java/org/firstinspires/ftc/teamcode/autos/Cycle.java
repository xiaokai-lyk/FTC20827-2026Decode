package org.firstinspires.ftc.teamcode.autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;
import org.firstinspires.ftc.teamcode.utils.AdaptivePIDController;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@Disabled
@Autonomous(name = "Cycle", group = "autos")
public class Cycle extends XKCommandOpmode {
    Hardwares hardwares;
    Drive drive;
    AutoDrive autoDrive;
    AdaptivePoseController adaptiveController;

    // Back-and-forth loop state
    private boolean goingToTarget = true; // true: go to target; false: return to start
    private int atGoalStableCount = 0;    // debounce cycles at goal
    private static final int AT_GOAL_STABLE_MIN = 5;

    // Start pose (captured on start) and target pose (constants)
    private double startXcm, startYcm, startHeadingDeg;
    private final double targetXcm = 100;
    private final double targetYcm = 50;
    private final double targetHeadingDeg = 180; // degrees

    // ----- PID tuning state for chassis (x/y/rotate) -----
    // axisIdx: 0 = xTrans, 1 = yTrans, 2 = rotate
    private int axisIdx = 0;
    // coefIdx: 0 = P, 1 = I, 2 = D
    private int coefIdx = 0;

    // previous button states for edge detection
    private boolean prevA = false, prevB = false, prevX = false, prevY = false;
    private boolean prevDpadUp = false, prevDpadDown = false, prevDpadLeft = false, prevDpadRight = false;

    // step sizes
    private final double smallStep = 0.01; // small increment
    private final double largeStep = 0.1;  // large increment

    @Override
    public void onStart() {
        // Capture the starting pose so we can return to it later
        if (hardwares != null && hardwares.sensors != null && hardwares.sensors.odo != null) {
            hardwares.sensors.odo.update();
            startXcm = hardwares.sensors.odo.getPosX(DistanceUnit.CM);
            startYcm = hardwares.sensors.odo.getPosY(DistanceUnit.CM);
            startHeadingDeg = hardwares.sensors.odo.getHeading(AngleUnit.DEGREES);
        }
        goingToTarget = true;
        atGoalStableCount = 0;
    }

    @Override
    public void run(){
        hardwares.sensors.odo.update();
        OdometerData odom = new OdometerData(hardwares.sensors.odo);

        // Select current goal based on state
        double goalX = goingToTarget ? targetXcm : startXcm;
        double goalY = goingToTarget ? targetYcm : startYcm;
        double goalHeadingDeg = goingToTarget ? targetHeadingDeg : startHeadingDeg; // AutoDrive expects degrees

        // Use adaptive PID control to drive toward current goal
        AutoDrive.Output out = autoDrive.driveToAdaptive(
            drive,
            adaptiveController,
            goalX, // 目标X (cm)
            goalY, // 目标Y (cm)
            goalHeadingDeg, // 目标heading (deg)
            odom,
            1,
            true
        );

        // Current pose
        double currX = hardwares.sensors.odo.getPosX(DistanceUnit.CM);
        double currY = hardwares.sensors.odo.getPosY(DistanceUnit.CM);
        double currHeadingDeg = hardwares.sensors.odo.getHeading(AngleUnit.DEGREES);

        // Errors
        double distanceErr = Math.hypot(goalX - currX, goalY - currY);
        double headingErrDeg = goalHeadingDeg - currHeadingDeg;
        // Normalize to (-180, 180]
        while (headingErrDeg > 180) headingErrDeg -= 360;
        while (headingErrDeg <= -180) headingErrDeg += 360;

        // Debounce arrival to avoid chatter
        if (out.atPosition && out.atHeading) {
            atGoalStableCount++;
        } else {
            atGoalStableCount = 0;
        }
        if (atGoalStableCount >= AT_GOAL_STABLE_MIN) {
            goingToTarget = !goingToTarget; // toggle direction to loop
            atGoalStableCount = 0;
        }

        // ----- Chassis PID tuning via gamepad -----
        // Controls (edge-triggered):
        // dpad_left/right : change axis (x/y/rotate)
        // dpad_up/down : change coefficient index (P/I/D)
        // A/B : increase/decrease by smallStep
        // Y/X : increase/decrease by largeStep

        // read current gamepad1 buttons
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        // change axis selection
        if (dpadLeft && !prevDpadLeft) axisIdx = (axisIdx - 1 + 3) % 3;
        if (dpadRight && !prevDpadRight) axisIdx = (axisIdx + 1) % 3;

        // change coefficient index
        if (dpadUp && !prevDpadUp) coefIdx = (coefIdx - 1 + 3) % 3;
        if (dpadDown && !prevDpadDown) coefIdx = (coefIdx + 1) % 3;

        // adjust values
        double delta = 0.0;
        if (a && !prevA) delta += smallStep;
        if (b && !prevB) delta -= smallStep;
        if (y && !prevY) delta += largeStep;
        if (x && !prevX) delta -= largeStep;

        if (Math.abs(delta) > 1e-9) {
            AdaptivePIDController pid;
            if (axisIdx == 0) pid = adaptiveController.getXTransPID();
            else if (axisIdx == 1) pid = adaptiveController.getYTransPID();
            else pid = adaptiveController.getRotatePID();

            double kp = pid.getKp();
            double ki = pid.getKi();
            double kd = pid.getKd();
            if (coefIdx == 0) kp += delta;
            else if (coefIdx == 1) ki += delta;
            else if (coefIdx == 2) kd += delta;
            pid.setPID(kp, ki, kd);
        }

        // update prev states
        prevA = a; prevB = b; prevX = x; prevY = y;
        prevDpadUp = dpadUp; prevDpadDown = dpadDown; prevDpadLeft = dpadLeft; prevDpadRight = dpadRight;

        // Telemetry for tuning
        AdaptivePIDController xpid = adaptiveController.getXTransPID();
        AdaptivePIDController ypid = adaptiveController.getYTransPID();
        AdaptivePIDController rpid = adaptiveController.getRotatePID();

        telemetry.addData("Axis", axisIdx == 0 ? "xTrans" : axisIdx == 1 ? "yTrans" : "rotate");
        telemetry.addData("Coef", coefIdx == 0 ? "P" : coefIdx == 1 ? "I" : "D");
        telemetry.addData("x PID", String.format("P=%.3f I=%.5f D=%.3f", xpid.getKp(), xpid.getKi(), xpid.getKd()));
        telemetry.addData("y PID", String.format("P=%.3f I=%.5f D=%.3f", ypid.getKp(), ypid.getKi(), ypid.getKd()));
        telemetry.addData("rotate PID", String.format("P=%.3f I=%.5f D=%.3f", rpid.getKp(), rpid.getKi(), rpid.getKd()));
        telemetry.addLine("Controls: dpad LR: axis | dpad UD: coef | A/B +/- small | Y/X +/- large");

        // Telemetry (existing)
        telemetry.addData("At Goal", out.atPosition && out.atHeading);
        telemetry.addData("阶段", goingToTarget ? "去目标" : "回起点");
        telemetry.addData("稳定计数", atGoalStableCount + "/" + AT_GOAL_STABLE_MIN);
        telemetry.addLine("=== Adaptive PID Debug ===");
        telemetry.addData("目标点 X(cm)", goalX);
        telemetry.addData("目标点 Y(cm)", goalY);
        telemetry.addData("目标角(deg)", goalHeadingDeg);
        telemetry.addData("当前位置 X(cm)", currX);
        telemetry.addData("当前位置 Y(cm)", currY);
        telemetry.addData("当前角(deg)", currHeadingDeg);
        telemetry.addData("误差 dx(cm)", out.dxCm);
        telemetry.addData("误差 dy(cm)", out.dyCm);
        telemetry.addData("距离误差(cm)", distanceErr);
        telemetry.addData("角度误差(deg)", headingErrDeg);
        telemetry.addData("atPosition", out.atPosition);
        telemetry.addData("atHeading", out.atHeading);
        telemetry.addLine("--- 输出 ---");
        telemetry.addData("xCmd", out.xCmd);
        telemetry.addData("yCmd", out.yCmd);
        telemetry.addData("rotateCmd", out.rotateCmd);
        telemetry.addLine("--- 轮速 ---");
        telemetry.addData("front left velocity", hardwares.motors.mFrontLeft.getVelocity());
        telemetry.addData("front right velocity", hardwares.motors.mFrontRight.getVelocity());
        telemetry.addData("back left velocity", hardwares.motors.mBackLeft.getVelocity());
        telemetry.addData("back right velocity", hardwares.motors.mBackRight.getVelocity());
        telemetry.update();
    }

    @Override
    public void initialize() {
        hardwares = new Hardwares(hardwareMap);
        drive = new Drive(hardwares);
        autoDrive = new AutoDrive();
        adaptiveController = Constants.PID.newPoseController();

        // nothing shooter-related here anymore
    }
}
