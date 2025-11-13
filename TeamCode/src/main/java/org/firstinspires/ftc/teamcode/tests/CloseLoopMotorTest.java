package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // added import

@Config
@TeleOp(name = "Close Loop Motor Test", group = "tests")
public class CloseLoopMotorTest extends LinearOpMode {
    public int velocity1 = 700;
    public int velocity2 = 1400;
    public double power = 1.0;
    public int threshold = 50; // speed error threshold
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "motor3"); // added motor3
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        preShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;
        boolean prevY = false;
        // added previous bumper states and enabled flags
        boolean prevLB = false;
        boolean prevRB = false;
        boolean motor12Enabled = false; // controls motor1 and motor2 on/off
        boolean motor3Enabled = true;  // controls motor3 on/off

        // added previous-trigger pressed flags for edge detection
        boolean prevLeftTriggerPressed = false;
        boolean prevRightTriggerPressed = false;

        // 新增 dpad 上下的前一帧状态变量
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        waitForStart();

        while (opModeIsActive()){
            final int THRESH_STEP = 10;
            // 读取当前 dpad 状态
            boolean currDpadUp = gamepad1.dpad_up;
            boolean currDpadDown = gamepad1.dpad_down;

            // dpad 上边缘检测
            if (currDpadUp && !prevDpadUp) {
                threshold += THRESH_STEP;
            }
            // dpad 下边缘检测
            if (currDpadDown && !prevDpadDown) {
                threshold -= THRESH_STEP;
            }
            if (threshold < 0) threshold = 0;

            // 更新 dpad 状态
            prevDpadUp = currDpadUp;
            prevDpadDown = currDpadDown;

            // 计算速度误差
            double err1 = Math.abs(motor1.getVelocity() - velocity1);
            double err2 = Math.abs(motor2.getVelocity() - velocity2);

            // 只有当 1、2 号电机都“到位”才允许 motor3 跑
            boolean motor3Allowed = (err1 <= threshold && err2 <= threshold);

            // 如果误差过大，强制关掉 motor3（无视右 bumper 开关）
            if (!motor3Allowed) {
                preShooter.setPower(0);
            } else {
                // 误差合格，按原来的开关量决定是否给 power
                preShooter.setPower(motor3Enabled ? power : 0);
            }

            boolean currA = gamepad1.a;
            boolean currB = gamepad1.b;
            boolean currX = gamepad1.x;
            boolean currY = gamepad1.y;

            // read bumpers for enable/disable toggles
            boolean currLB = gamepad1.left_bumper;
            boolean currRB = gamepad1.right_bumper;

            // read analog triggers (0..1)
            float leftTrigger = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;

            // replace continuous scaling with rising-edge detection:
            // consider a trigger "pressed" when it exceeds this threshold
            final float TRIGGER_PRESS_THRESHOLD = 0.5f;
            // step to change power on each rising edge
            final double POWER_STEP = 0.05;

            boolean leftPressed = leftTrigger > TRIGGER_PRESS_THRESHOLD;
            boolean rightPressed = rightTrigger > TRIGGER_PRESS_THRESHOLD;

            // on rising edge of right trigger increase power
            if (rightPressed && !prevRightTriggerPressed) {
                power = Math.min(1.0, power + POWER_STEP);
            }
            // on rising edge of left trigger decrease power
            if (leftPressed && !prevLeftTriggerPressed) {
                power = Math.max(0.0, power - POWER_STEP);
            }

            // update previous trigger pressed states for next loop
            prevLeftTriggerPressed = leftPressed;
            prevRightTriggerPressed = rightPressed;

            if (currA && !prevA) {
                velocity1 += 50;
            }
            if (currB && !prevB) {
                velocity1 -= 50;
            }
            if (currX && !prevX) {
                velocity2 += 50;
            }
            if (currY && !prevY) {
                velocity2 -= 50;
            }

            // edge-detect bumpers to toggle enables
            if (currLB && !prevLB) {
                motor12Enabled = !motor12Enabled;
            }
            if (currRB && !prevRB) {
                motor3Enabled = !motor3Enabled;
            }

            prevA = currA;
            prevB = currB;
            prevX = currX;
            prevY = currY;
            prevLB = currLB;
            prevRB = currRB;

            if (velocity1 < 0) velocity1 = 0;
            if (velocity2 < 0) velocity2 = 0;

            // apply motor1/2 based on enabled flag
            if (motor12Enabled) {
                motor1.setVelocity(velocity1);
                motor2.setVelocity(velocity2);
            } else {
                motor1.setVelocity(0);
                motor2.setVelocity(0);
            }


            telemetry.addData("Velocity1", velocity1);
            telemetry.addData("motor1 Velocity", motor1.getVelocity());
            telemetry.addData("Velocity2", velocity2);
            telemetry.addData("motor2 Velocity", motor2.getVelocity());
            telemetry.addData("motor3 Velocity", preShooter.getVelocity());

            // show enable states and power
            telemetry.addData("motor1+2 Enabled", motor12Enabled);
            telemetry.addData("motor3 Enabled", motor3Enabled);
            telemetry.addData("motor3 Power (set)", power);
            telemetry.addData("leftTrigger", leftTrigger);
            telemetry.addData("rightTrigger", rightTrigger);

            telemetry.addData("speed threshold", threshold);
            telemetry.addData("err1", err1);
            telemetry.addData("err2", err2);
            telemetry.addData("motor3 allowed", motor3Allowed);

            telemetry.update();
        }
    }
}
