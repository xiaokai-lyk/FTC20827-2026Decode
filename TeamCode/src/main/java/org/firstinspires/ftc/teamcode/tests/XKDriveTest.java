package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.drive.*;
import java.util.function.Supplier;

@TeleOp(name = "XKDriveTest", group = "Tests")
public class XKDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 硬件初始化
        DcMotorEx fl = (DcMotorEx)hardwareMap.dcMotor.get("fl");
        DcMotorEx fr = (DcMotorEx)hardwareMap.dcMotor.get("fr");
        DcMotorEx rl = (DcMotorEx)hardwareMap.dcMotor.get("rl");
        DcMotorEx rr = (DcMotorEx)hardwareMap.dcMotor.get("rr");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx[] motors = new DcMotorEx[]{fl, fr, rl, rr};
        // 2. PID与前馈参数（示例参数，需根据实际调试）
        XKMecanumDrive drive = initializeDrive(motors);
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
        // 3. 传感器/目标/编码器接口（示例：用手柄控制目标点，传感器数据用假数据/需替换为实际接口）
        XKMecanumDrive.Pose2d currentPose = new XKMecanumDrive.Pose2d(otos.getPosition().x, otos.getPosition().y, otos.getPosition().h);
        XKMecanumDrive.Velocity2d currentVel = new XKMecanumDrive.Velocity2d(otos.getVelocity().x, otos.getVelocity().y, otos.getVelocity().h);
        XKMecanumDrive.Accel2d currentAccel = new XKMecanumDrive.Accel2d(otos.getAcceleration().x, otos.getAcceleration().y, otos.getAcceleration().h);
        double[] wheelSpeeds = new double[4];
        waitForStart();
        while (opModeIsActive()) {
            // 示例：用手柄左摇杆控制目标x/y，右摇杆x控制目标theta
            double targetX = gamepad1.left_stick_x * 10;
            double targetY = gamepad1.left_stick_y * 10;
            double targetTheta = gamepad1.right_stick_x;
            XKMecanumDrive.Pose2d targetPose = new XKMecanumDrive.Pose2d(targetX, targetY, targetTheta);
            // 传感器接口（实际应替换为真实传感器数据）
            Supplier<XKMecanumDrive.Pose2d> currentPoseSupplier = () -> currentPose;
            Supplier<XKMecanumDrive.Velocity2d> currentVelSupplier = () -> currentVel;
            Supplier<XKMecanumDrive.Accel2d> currentAccelSupplier = () -> currentAccel;
            Supplier<double[]> wheelSpeedsSupplier = () -> wheelSpeeds;
            for (int i = 0; i < 4; i++) {
                wheelSpeeds[i] = motors[i].getVelocity();
            }
            // 调用主控制循环
            drive.update(targetPose, currentPoseSupplier, currentVelSupplier, currentAccelSupplier, wheelSpeedsSupplier);
            telemetry.addData("Target", "x:%.2f y:%.2f th:%.2f", targetX, targetY, targetTheta);
            telemetry.addLine("==========================");
            // 添加调试信息：电机速度
            telemetry.addData("FL Speed", "%.2f", wheelSpeeds[0]);
            telemetry.addData("FR Speed", "%.2f", wheelSpeeds[1]);
            telemetry.addData("RL Speed", "%.2f", wheelSpeeds[2]);
            telemetry.addData("RR Speed", "%.2f", wheelSpeeds[3]);
            // 添加调试信息：当前位姿
            telemetry.addData("Current Pose", "x:%.2f y:%.2f th:%.2f", currentPose.x, currentPose.y, currentPose.theta);
            telemetry.update();
            sleep(20);
        }
    }

    @NonNull
    private static XKMecanumDrive initializeDrive(DcMotorEx[] motors) {
        PIDController posPidX = new PIDController(1.0, 0.0, 0.1);
        PIDController posPidY = new PIDController(1.0, 0.0, 0.1);
        PIDController posPidTheta = new PIDController(2.0, 0.0, 0.2);
        PIDController speedPidFL = new PIDController(0.5, 0.0, 0.05);
        PIDController speedPidFR = new PIDController(0.5, 0.0, 0.05);
        PIDController speedPidRL = new PIDController(0.5, 0.0, 0.05);
        PIDController speedPidRR = new PIDController(0.5, 0.0, 0.05);
        double L = 0.18; // 机器人半长(m)
        double W = 0.15; // 机器人半宽(m)
        double mass = 1; // 质量(kg)
        double inertia = (1.0 / 12.0) * mass * (L * L + W * W);; // 转动惯量(kg*m^2)
        FeedforwardController ffController = new FeedforwardController(mass, inertia, 0.3);
        return new XKMecanumDrive(L, W,
                posPidX, posPidY, posPidTheta,
                speedPidFL, speedPidFR, speedPidRL, speedPidRR,
                ffController, motors);
    }
}
