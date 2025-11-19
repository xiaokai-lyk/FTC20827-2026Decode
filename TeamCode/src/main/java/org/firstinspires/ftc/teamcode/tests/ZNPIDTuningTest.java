package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardwares;

/**
 * Ziegler-Nichols (ZN) PID 整定测试OpMode
 *
 * 【使用方法】
 * 1. 上传并运行此OpMode到机器人控制器。
 * 2. 启动后，使用A/B键增加/减少Kp（比例增益）。
 * 3. 使用X键停止电机，Y键启动目标速度（建议设置为较高值，便于观察振荡）。
 * 4. 观察Telemetry中的电机速度和Kp值，逐步增加Kp，直到电机速度出现持续振荡（不衰减也不发散）。
 * 5. 当出现持续振荡时，代码会自动测量并显示振荡周期Tu和当前Kp（即Ku）。
 * 6. 记录下Tu和Ku，使用ZN公式计算PID参数。
 *
 * 【ZN术语】
 * Ku = 临界增益（刚好出现持续振荡时的比例增益）
 * Tu = 临界振荡周期（单位：秒）
 * Ti = 积分时间常数（秒）  Td = 微分时间常数（秒）
 * Ki = Kp / Ti             Kd = Kp * Td
 *
 * 【三套参考参数公式】
 * 1) 经典(standard)：响应较快，可能有一定超调
 *    P   : Kp = 0.50 Ku
 *    PI  : Kp = 0.45 Ku, Ti = Tu / 1.2        => Ki = Kp / Ti
 *    PID : Kp = 0.60 Ku, Ti = 0.50 Tu, Td=0.125 Tu => Ki = Kp / (0.50 Tu), Kd = Kp * (0.125 Tu)
 *
 * 2) 保守(conservative)：降低超调和振铃
 *    PID : Kp = 0.33 Ku, Ti = 0.50 Tu, Td = 0.33 Tu => Ki = Kp / (0.50 Tu), Kd = Kp * (0.33 Tu)
 *
 * 3) 激进(aggressive)：追求最快的阶跃响应，允许更高超调
 *    PID : Kp = 0.70 Ku, Ti = 0.40 Tu, Td = 0.15 Tu => Ki = Kp / (0.40 Tu), Kd = Kp * (0.15 Tu)
 *    使用激进参数后若超调太大，可逐步减小Kp或增大Ti。
 *
 * 【离散实现公式（循环周期 dt 秒）】
 * 误差 e_n = (目标��度 - 实际速度)
 * 积分 I_n = I_{n-1} + e_n * dt
 * 微分 D_n = (e_n - e_{n-1}) / dt
 * 输出功率 u_n = Kp * e_n + Ki * I_n + Kd * D_n + Kf * V_target
 *
 * 若某些库内部已乘 dt：可能需要使用 Ki' = Ki * dt, Kd' = Kd / dt。
 * FTC DcMotorEx 的 PIDF 系数（p,i,d,f）默认针对速度误差直接使用，不需再额外乘 dt（SDK内部处理）。因此直接使用计算出的 Ki, Kd。
 *
 * 【前馈 F (Kf) 计算】
 * 目标：在无误差时由 F 提供主要输出，让PID只做微调。
 * 基本估算：Kf ≈ 1 / Vmax （Vmax = 空载最大稳定速度 ticks/s）
 * 经验法：设定某目标速度 Vt，调试一个功率 P 使其稳定，Kf ≈ P / Vt
 * 线性拟合法：多组 (功率P_i, 速度V_i) 数据，拟合 P = a * V + b，Kf ≈ a （忽略截距 b 或将 b 视为静摩擦补偿）。
 * 若存在明显静摩擦，可在低速区添加最小输出 bias：u = bias + PIDF，bias 仅在 |V_target| > 0 时施加。
 *
 * 【示例计算】(已测 Ku=1.20, Tu=0.40)
 * 经典PID: Kp=0.72, Ki=0.72/(0.50*0.40)=3.60, Kd=0.72*(0.125*0.40)=0.036
 * 保守PID: Kp=0.396, Ki=0.396/(0.50*0.40)=1.98, Kd=0.396*(0.33*0.40)≈0.0523
 * 激进PID: Kp=0.84, Ki=0.84/(0.40*0.40)=5.25, Kd=0.84*(0.15*0.40)=0.0504
 *
 * 【实操建议】
 * 1. 先设定 Kf，使电机在无PID或极小Kp时能基本达到目标速度。
 * 2. 增加 Kp 直到出现持续振荡 -> 得到 Ku, Tu。
 * 3. 用上表选一套参数（推荐先经典，再微调）。
 * 4. 若超调过大：减小 Kp 或增大 Ti（降低 Ki）；若响应太慢：稍增 Kp 或减小 Ti。
 * 5. 调试中监控电机温度、电流与机械安全。
 */
@TeleOp(name = "ZN PID Tuning Test", group = "tests")
public class ZNPIDTuningTest extends LinearOpMode {
    // 可调参数
    private double Kp; // 初始比例增益
    private double Ki; // 初始积分增益
    private double Kd; // 初始微分增益
    private double Kf; // 前馈增益，建议F=1.0/最大速度
    private final double KpStep = 0.05; // 增益调整步长
    private final int targetVelocity = 1200; // 目标速度（可根据实际情况调整）
    private final int sampleWindow = 10; // 采样窗口（ms）
    private final int minOscillations = 3; // 至少检测到几次振荡才判定为临界振荡

    @Override
    public void runOpMode() {
        DcMotorEx motor = new Hardwares(hardwareMap).motors.shooterFront;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        boolean prevA = false, prevB = false, prevX = false, prevY = false;
        boolean running = false;
        long lastSampleTime = 0;
        boolean lastAbove = false;
        int oscillationCount = 0;
        long firstCrossTime = 0;
        long lastCrossTime = 0;
        double Tu = 0;
        double Ku = 0;
        boolean oscillating = false;
        Kp = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
        Ki = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
        Kd = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
        Kf = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;

        telemetry.addLine("ZN PID Tuning Test\nA/B: Kp +/-\nY: 启动目标速度\nX: 停止\n观察振荡，记录Tu和Kp");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean currA = gamepad1.a;
            boolean currB = gamepad1.b;
            boolean currX = gamepad1.x;
            boolean currY = gamepad1.y;

            // 调整Kp
            boolean kpChanged = false;
            if (currA && !prevA) { Kp += KpStep; kpChanged = true; }
            if (currB && !prevB) { Kp = Math.max(0, Kp - KpStep); kpChanged = true; }
            if (kpChanged) {
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(Kp, Ki, Kd, Kf));
            }

            // 启动/停止
            if (currY && !prevY) {
                running = true;
                motor.setVelocity(targetVelocity);
                oscillationCount = 0;
                firstCrossTime = 0;
                lastCrossTime = 0;
                oscillating = false;
            }
            if (currX && !prevX) {
                running = false;
                motor.setVelocity(0);
                oscillationCount = 0;
                firstCrossTime = 0;
                lastCrossTime = 0;
                oscillating = false;
            }

            // 振荡检测
            if (running) {
                double velocity = motor.getVelocity();
                long now = System.currentTimeMillis();
                if (now - lastSampleTime > sampleWindow) {
                    boolean above = velocity > targetVelocity;
                    if (oscillationCount == 0) lastAbove = above;
                    if (above != lastAbove) {
                        if (oscillationCount == 0) {
                            firstCrossTime = now;
                        } else {
                            lastCrossTime = now;
                        }
                        oscillationCount++;
                        lastAbove = above;
                    }
                    lastSampleTime = now;
                }
                if (oscillationCount >= minOscillations && !oscillating) {
                    Tu = (lastCrossTime - firstCrossTime) / (double)(oscillationCount - 1) / 1000.0; // 秒
                    Ku = Kp;
                    oscillating = true;
                }
            }

            // 实时显示信息
            telemetry.addData("当前PIDF参数", "Kp=%.4f Ki=%.4f Kd=%.4f Kf=%.5f", Kp, Ki, Kd, Kf);
            telemetry.addData("目标速度", targetVelocity);
            telemetry.addData("当前速度", motor.getVelocity());
            telemetry.addData("运行状态", running ? "运行中" : "已停止");
            telemetry.addData("振荡次数", oscillationCount);
            if (oscillating) {
                telemetry.addData("检测到临界振荡! Ku=", Ku);
                telemetry.addData("振荡周期 Tu (s)", Tu);
                telemetry.addLine("请记录Ku和Tu, 用ZN公式计算PID参数");

                double classicKp = 0.60 * Ku;
                double classicKi = classicKp / (0.50 * Tu);
                double classicKd = classicKp * (0.125 * Tu);
                double conservativeKp = 0.33 * Ku;
                double conservativeKi = conservativeKp / (0.50 * Tu);
                double conservativeKd = conservativeKp * (0.33 * Tu);
                double aggressiveKp = 0.70 * Ku;
                double aggressiveKi = aggressiveKp / (0.40 * Tu);
                double aggressiveKd = aggressiveKp * (0.15 * Tu);
                telemetry.addLine("=== 推荐PID参数 (根据Ku,Tu) ===");
                telemetry.addData("经典 PID", "Kp=%.3f Ki=%.3f Kd=%.3f", classicKp, classicKi, classicKd);
                telemetry.addData("保守 PID", "Kp=%.3f Ki=%.3f Kd=%.3f", conservativeKp, conservativeKi, conservativeKd);
                telemetry.addData("激进 PID", "Kp=%.3f Ki=%.3f Kd=%.3f", aggressiveKp, aggressiveKi, aggressiveKd);
                telemetry.addLine("激进参数可能导致较大超调, 建议逐步试验并监控温度与电流");
            }
            telemetry.update();

            prevA = currA;
            prevB = currB;
            prevX = currX;
            prevY = currY;
        }
        // 停止电机
        motor.setVelocity(0);
    }
}
