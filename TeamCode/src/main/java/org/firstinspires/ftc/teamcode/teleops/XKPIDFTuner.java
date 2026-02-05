package org.firstinspires.ftc.teamcode.teleops;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.XKPIDFController;

/**
 * 专门用于调节 XKPIDFController 参数的 OpMode。
 * 使用 FTC Dashboard 进行实时调参和波形观察。
 * <h1>注意安全：确保其他物体不要接触到飞轮</h1>
 * <h1>不要在飞轮高速运动时骤减Target Velocity(e.g. 2500->0)可能会导致结构损坏或危险！</h1>
 */
@Config
@TeleOp(name = "XK Shooter Tuner", group = "Tuning")
public class XKPIDFTuner extends LinearOpMode {

    // =====================================
    // Dashboard 可调节参数
    // =====================================
    public static double TARGET_VELOCITY = 0; // 目标速度

    // PIDF 参数
    public static double kP = 0.0003;
    public static double kI = 0.0001;
    public static double kD = 0.0;
    public static double kF = 0.00037; // 核心：前馈系数

    /*
     * 调参电压提示：
     * 不需要刻意将电池控制在 12V。建议使用满电（>13V）电池进行调参。
     * 控制器会自动将您调好的参数"归一化"到 12V 标准。
     * 只需避免在低压（<12V）下调参，因为电机可能物理功率不足，导致参数虚假。
     */

    // Bang-Bang 助力参数
    public static boolean ENABLE_BANG_BANG = true;
    public static double BANG_BAND_ABS = 30.0; // 绝对误差带宽 (ticks/s)
    public static double ASSIST_POWER = 1.0;   // 助力功率
    public static double PITCH_ANGLE_DEG = 0.0; // 俯仰角度
    public static double GATE_OPEN_ANGLE_DEG = 50; // 挡板打开角度
    public static double GATE_CLOSE_ANGLE_DEG = 107; // 挡板关闭角度
    public static boolean GATE_OPEN = false; // 挡板状态
    public static boolean ENABLE_INTAKE = false; // 是否启用吸球电机
    public static boolean ENABLE_SHOOTER = false;

    // 控制器与电机
    // 同轴双电机使用同一个控制器，避免相互"打架"
    private XKPIDFController controller;
    private DcMotorEx shooterLeft, shooterRight;
    private Intake intake;
    private ServoEx pitch, gate;


    @Override
    public void runOpMode() {
        // 1. 初始化 Dashboard 遥测，这样 telemetery.addData 的内容会自动发给网页端画图
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. 初始化硬件 (直接从 hardwareMap 获取更纯粹，避开 Hardwares 类中可能的额外初始化耗时)
        // 也可以使用 new Hardwares(hardwareMap) 如果你习惯那样
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        pitch = new SimpleServo(hardwareMap, "pitch", 0, 300);
        pitch.turnToAngle(PITCH_ANGLE_DEG, AngleUnit.DEGREES);

        gate = new SimpleServo(hardwareMap, "gate", 0, 300);

        // 电机配置
        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);  // 根据 Shooter.java 的配置
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        // 重要：自定义闭环必须使用 RUN_WITHOUT_ENCODER
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 3. 初始化控制器
        // 获取电压传感器通常是找到第一个可用的
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 两个电机共轴，只需要一个控制器计算 Power
        controller = new XKPIDFController(voltageSensor);

        // intake 初始化
        intake = new Intake(new Hardwares(hardwareMap));
        intake.setIntakePower(1.0);

        telemetry.addLine("Ready. Open FTC Dashboard (http://192.168.43.1:8080/dash) to tune.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 4. 实时更新参数 (从 Dashboard 读取)
            updateControllerParams(controller);


            // 5. 获取状态并计算
            double velLeft = shooterLeft.getVelocity();
            double velRight = shooterRight.getVelocity();

            // 取平均速度作为反馈（如果共轴，这样更稳健）
            // 如果不是共轴而是双飞轮结构，请改回分别控制
            double avgVel = (velLeft + velRight) / 2.0;

            // 只计算一次闭环
            XKPIDFController.Output output = controller.update(avgVel);

            // 6. 应用 Power (两个电机给一样的力)
            if (ENABLE_SHOOTER){
                shooterLeft.setPower(output.power);
                shooterRight.setPower(output.power);
            }else{
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }

            // 吸球电机
            if (ENABLE_INTAKE) {
                intake.startIntake().schedule();
            } else {
                intake.stopIntake().schedule();
            }

            if(GATE_OPEN){
                gate.turnToAngle(GATE_OPEN_ANGLE_DEG, AngleUnit.DEGREES);
            }else {
                gate.turnToAngle(GATE_CLOSE_ANGLE_DEG, AngleUnit.DEGREES);
            }

            pitch.turnToAngle(PITCH_ANGLE_DEG, AngleUnit.DEGREES);

            // 7. 发送遥测数据用于绘图
            // 电压显示
            telemetry.addData("Battery Voltage", voltageSensor.getVoltage());

            // 目标线
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            // 实际曲线
            telemetry.addData("Vel Left", velLeft);
            telemetry.addData("Vel Right", velRight);
            telemetry.addData("Vel Avg", avgVel);

            telemetry.addData("pitch angle(deg)", pitch.getAngle(AngleUnit.DEGREES));

            // 功率曲线 (方便观察是否饱和或震荡)
            telemetry.addData("Power Out", output.power);
            // Bang-Bang 激活状态 (乘 1000 方便在速度图中看出来)
            telemetry.addData("BangBang Active", output.isAssistActive);

            telemetry.update();
        }

        // 结束时停机
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    private void updateControllerParams(@NonNull XKPIDFController ctrl) {
        ctrl.setPIDF(kP, kI, kD, kF)
            .setBangBang(ENABLE_BANG_BANG, BANG_BAND_ABS, ASSIST_POWER)
            .setTargetVelocity(TARGET_VELOCITY);
    }
}
