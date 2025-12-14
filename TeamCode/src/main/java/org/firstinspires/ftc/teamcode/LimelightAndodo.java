package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Limelight&odo - PosMode", group = "TeleOp")
public class LimelightAndodo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(7);
        limelight.start();

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // 初始位置设定点
        double resetPosX = -300;
        double resetPosY = 0;
        odo.setPosition(new Pose2D(DistanceUnit.CM, resetPosX, resetPosY, AngleUnit.DEGREES, 0));

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx pan = hardwareMap.get(DcMotorEx.class, "pan");

        // 设置刹车行为
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 设置电机方向
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // 初始化云台编码器并切换到RUN_TO_POSITION模式
        pan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 调整参数 (在机器人上调整)
        final double TICKS_PER_DEGREE = 72.1;      // 每度编码器脉冲数 (需校准)
        final double TX_DEGREE_THRESHOLD = 1.0;    // 只有当 |tx| > 阈值时才命令 (度)

        // 机械限制: 设置宽泛默认值 (改为适合机器人的限制)
        final int MIN_PAN_TICKS = -12960;          // 机械最小编码器脉冲 (-180度)
        final int MAX_PAN_TICKS = 12960;           // 机械最大编码器脉冲 (+180度)

        // PID 调校 (用于计算目标位置，而非速度)
        final double KP = 0.05;    // 比例增益（用于目标角度修正）
        final double KI = 0.0;
        final double KD = 0.0;

        final PIDFController anglePid = new PIDFController(KP, KI, KD, 0);
        anglePid.setSetPoint(0.0);

        final double MAX_SPEED = 0.6; // 最大电机功率

        boolean autoPanEnabled = true;            // 开始时启用自动云台
        long lastToggleTime = 0;
        boolean goToAprilTagMode = false;         // 是否进入前往AprilTag模式

        // AprilTag位置数据（示例：假设AprilTag在场地中心）
        final double APRIL_TAG_X = 0; // AprilTag的X坐标 (厘米)
        final double APRIL_TAG_Y = 0; // AprilTag的Y坐标 (厘米)

        telemetry.addData(">", "准备就绪 - A切换自动云台, B归位, Y重置ODO, X前往AprilTag");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // 通过D-pad Down重置ODO位置到预设值
            if (gamepad1.dpad_down) {
                odo.setPosition(new Pose2D(DistanceUnit.CM, resetPosX, resetPosY, AngleUnit.DEGREES, 0));
                telemetry.addData("ODO", "已重置到预设位置 (" + resetPosX + ", " + resetPosY + ")");
            }

            // 通过Y键重置ODO位置到当前位置
            if (gamepad1.y && (System.currentTimeMillis() - lastToggleTime) > 300) {
                // 获取当前机器人位置
                Pose2D currentPose = odo.getPosition(); // 注意：使用getPosition()
                if (currentPose != null) {
                    resetPosX = currentPose.getX(DistanceUnit.CM);
                    resetPosY = currentPose.getY(DistanceUnit.CM);
                    telemetry.addData("ODO", "已更新重置位置 (" + resetPosX + ", " + resetPosY + ")");
                }
                lastToggleTime = System.currentTimeMillis();
            }

            // 驱动控制 (保持原样)
            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((x + y + rx) / denominator);
            backLeft.setPower((x - y + rx) / denominator);
            frontRight.setPower((x - y - rx) / denominator);
            backRight.setPower((x + y - rx) / denominator);

            // 通过A键切换自动云台 (防抖)
            if (gamepad1.a && (System.currentTimeMillis() - lastToggleTime) > 300) {
                autoPanEnabled = !autoPanEnabled;
                goToAprilTagMode = false; // 退出前往AprilTag模式
                lastToggleTime = System.currentTimeMillis();
            }

            // 通过B键归位云台 (重置编码器到0)
            if (gamepad1.b && (System.currentTimeMillis() - lastToggleTime) > 300) {
                pan.setTargetPosition(0);
                pan.setPower(MAX_SPEED);
                goToAprilTagMode = false; // 退出前往AprilTag模式
                lastToggleTime = System.currentTimeMillis();
            }

            // 通过X键进入前往AprilTag模式
            if (gamepad1.x && (System.currentTimeMillis() - lastToggleTime) > 300) {
                goToAprilTagMode = !goToAprilTagMode;
                lastToggleTime = System.currentTimeMillis();
            }

            // 获取Limelight数据 (可能为空)
            LLResult res = limelight.getLatestResult();
            if (res != null) {
                telemetry.addData("tx", res.getTx());
                telemetry.addData("ty", res.getTy());
                telemetry.addData("ta", res.getTa());
                telemetry.addData("Latency", res.getCaptureLatency());
            } else {
                telemetry.addData("Limelight", "无结果 (null)");
            }

            // 获取当前机器人姿态
            Pose2D robotPose = odo.getPosition(); // 注意：使用getPosition()
            double robotAngle = 0;
            if (robotPose != null) {
                robotAngle = robotPose.getHeading(AngleUnit.DEGREES);
                telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, H: %.2f",
                    robotPose.getX(DistanceUnit.CM),
                    robotPose.getY(DistanceUnit.CM),
                    robotAngle);
            }

            // 云台控制逻辑
            int targetPanPosition = pan.getTargetPosition();
            int currentPanPosition = pan.getCurrentPosition();

            if (goToAprilTagMode && robotPose != null) {
                // 基于机器人位置和预设AprilTag位置计算所需角度

                double deltaX = APRIL_TAG_X - robotPose.getX(DistanceUnit.CM);
                double deltaY = APRIL_TAG_Y - robotPose.getY(DistanceUnit.CM);

                // 计算目标角度 (相对于机器人前方)
                double targetAngle = Math.atan2(deltaY, deltaX); // 弧度
                targetAngle = Math.toDegrees(targetAngle);       // 转换为度

                // 考虑机器人当前朝向
                double absoluteTargetAngle = targetAngle - robotAngle;

                // 标准化角度到 [-180, 180]
                while (absoluteTargetAngle > 180) absoluteTargetAngle -= 360;
                while (absoluteTargetAngle < -180) absoluteTargetAngle += 360;

                // 转换为目标编码器位置
                targetPanPosition = (int)(absoluteTargetAngle * TICKS_PER_DEGREE);

                // 限制在允许范围内
                if (targetPanPosition > MAX_PAN_TICKS) targetPanPosition = MAX_PAN_TICKS;
                if (targetPanPosition < MIN_PAN_TICKS) targetPanPosition = MIN_PAN_TICKS;

                // 设置目标位置
                pan.setTargetPosition(targetPanPosition);
                pan.setPower(MAX_SPEED);

                telemetry.addData("GoToAprilTag", "Target Angle: %.2f", absoluteTargetAngle);
                telemetry.addData("Target Pos", targetPanPosition);

                // 如果接近目标，则等待Limelight接管
                if (Math.abs(targetPanPosition - currentPanPosition) < 50) { // 误差阈值
                    goToAprilTagMode = false;
                    autoPanEnabled = true; // 启用自动跟踪
                    telemetry.addData("Status", "接近目标，切换到自动跟踪");
                }
            }
            else if (autoPanEnabled && res != null && res.isValid()) {
                double txDeg = res.getTx(); // 正值 = 目标在右侧
                telemetry.addData("tx(deg)", String.format("%.2f", txDeg));

                if (Math.abs(txDeg) > TX_DEGREE_THRESHOLD) {
                    // 使用PID控制器计算角度修正
                    double angleCorrection = anglePid.calculate(-txDeg);

                    // 计算新的目标位置
                    double currentAngle = currentPanPosition / TICKS_PER_DEGREE;
                    double newTargetAngle = currentAngle + angleCorrection;

                    // 标准化角度到 [-180, 180]
                    while (newTargetAngle > 180) newTargetAngle -= 360;
                    while (newTargetAngle < -180) newTargetAngle += 360;

                    // 转换为目标编码器位置
                    targetPanPosition = (int)(newTargetAngle * TICKS_PER_DEGREE);

                    // 限制在允许范围内
                    if (targetPanPosition > MAX_PAN_TICKS) targetPanPosition = MAX_PAN_TICKS;
                    if (targetPanPosition < MIN_PAN_TICKS) targetPanPosition = MIN_PAN_TICKS;

                    // 实现360度连续旋转：当接近极限时快速旋转到另一侧
                    if (targetPanPosition >= MAX_PAN_TICKS - 500 && txDeg < 0) {
                        // 如果目标在左侧但我们在右极限，跳转到左极限
                        targetPanPosition = MIN_PAN_TICKS + (int)((180 + newTargetAngle) * TICKS_PER_DEGREE);
                    }
                    else if (targetPanPosition <= MIN_PAN_TICKS + 500 && txDeg > 0) {
                        // 如果目标在右侧但我们在左极限，跳转到右极限
                        targetPanPosition = MAX_PAN_TICKS - (int)((180 - newTargetAngle) * TICKS_PER_DEGREE);
                    }

                    // 设置目标位置
                    pan.setTargetPosition(targetPanPosition);
                    pan.setPower(MAX_SPEED);

                    telemetry.addData("Target Pos", targetPanPosition);
                    telemetry.addData("Angle Correction", angleCorrection);
                } else {
                    // 在死区内 -> 保持当前位置
                    pan.setPower(0);
                    anglePid.reset();
                }
            } else {
                // 手动控制云台（使用右摇杆）
                double manualPan = -gamepad1.right_stick_x;
                if (Math.abs(manualPan) > 0.1) {
                    // 计算手动目标位置
                    int manualTarget = currentPanPosition + (int)(manualPan * 200); // 调整灵敏度

                    // 限制在允许范围内
                    if (manualTarget > MAX_PAN_TICKS) manualTarget = MAX_PAN_TICKS;
                    if (manualTarget < MIN_PAN_TICKS) manualTarget = MIN_PAN_TICKS;

                    pan.setTargetPosition(manualTarget);
                    pan.setPower(MAX_SPEED);
                } else {
                    // 保持当前位置
                    pan.setPower(0);
                }

                telemetry.addData("AutoPan", autoPanEnabled ?
                    (goToAprilTagMode ? "GoToAprilTag mode" : "enabled (no valid target)") :
                    "disabled");
            }

            // 显示云台状态
            telemetry.addData("PanPos", currentPanPosition);
            telemetry.addData("TargetPos", pan.getTargetPosition());
            telemetry.addData("RunMode", pan.getMode().toString());
            telemetry.addData("TicksPerDegree", String.format("%.2f", TICKS_PER_DEGREE));
            telemetry.addData("GoToAprilTag", goToAprilTagMode ? "Active" : "Inactive");
            telemetry.update();
        }
    }
}
