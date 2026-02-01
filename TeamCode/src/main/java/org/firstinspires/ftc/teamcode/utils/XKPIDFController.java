package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 全功能 PIDF + Bang-Bang 速度控制器。
 * 直接计算电机 Power 值，替代 SDK 的 RUN_USING_ENCODER。
 * 包含：
 * - 完整的 PIDF 闭环计算
 * - 电压补偿 (使得 PIDF 参数在不同电压下表现一致)
 * - Bang-Bang 助力 (应对严重掉速)
 *
 * <h3>调参步骤指南：</h3>
 * <ol>
 *   <li><b>准备工作：</b>
 *      <ul>
 *          <li>将电机模式设为 RUN_WITHOUT_ENCODER。</li>
 *          <li>确保电压补偿正常工作 (传入正确的 VoltageSensor)。</li>
 *          <li>暂时关闭 Bang-Bang (enabled=false) 和积分 (kI=0)、微分 (kD=0)、比例 (kP=0)。</li>
 *      </ul>
 *   </li>
 *   <li><b>标定 kF (前馈)：</b>
 *      <ul>
 *          <li>给定一个常用目标速度 (如 2000 ticks/s)。</li>
 *          <li>逐步调整 kF，直到电机实际稳定速度接近目标速度 (误差 &lt; 5-10%)。</li>
 *          <li>估算公式：kF ≈ 1.0 / 理论最大空载速度 (ticks/s)。</li>
 *      </ul>
 *   </li>
 *   <li><b>调整 kP (比例)：</b>
 *      <ul>
 *          <li>增加 kP，观察系统对扰动的响应。</li>
 *          <li>目标是让速度能快速对齐目标，且没有明显的持续震荡。</li>
 *      </ul>
 *   </li>
 *   <li><b>调整 Bang-Bang (助力)：</b>
 *      <ul>
 *          <li>开启 Bang-Bang (enabled=true)。</li>
 *          <li>调整 bandAbs (例如 50-100 ticks/s)：当跌落超过此值时触发助力。</li>
 *          <li>调整 assistPower (0.5-1.0)：给一个足够大的力让飞轮迅速回升，但尽量不要引起过大的过冲。</li>
 *      </ul>
 *   </li>
 *   <li><b>(可选) 调整 kI (积分)：</b>
 *      <ul>
 *          <li>如果稳态误差消除不掉，给予极小的 kI。</li>
 *          <li>注意：Shooter 这种大惯量系统加积分容易震荡，需谨慎。</li>
 *      </ul>
 *   </li>
 * </ol>
 */
public class XKPIDFController  {

    public static class Output {
        /**
         * 最终计算出的电机功率 (-1.0 ~ 1.0)。
         * 直接以此值调用 motor.setPower()。
         */
        public final double power;

        /**
         * 仅仅用于调试：是否处于 Bang-Bang 助力状态
         */
        public final boolean isAssistActive;

        public Output(double power, boolean isAssistActive) {
            this.power = power;
            this.isAssistActive = isAssistActive;
        }
    }

    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();

    // PIDF Constants (tuned for 12V)
    private double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0;
    private double nominalVoltage = 12.0;

    // State
    private double targetVel = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean isFirstRun = true;

    // Bang-Bang config
    private boolean bangBangEnabled = true;
    private double bandAbs = 30.0;
    private double assistPower = 1.0;

    // Output clamps
    private double integralLimit = 100000.0; // Prevent integral windup

    // 纳秒时间戳，用于高精度 dt 计算
    private long lastNanoTime = 0;

    public XKPIDFController(@NonNull VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    /**
     * 设置 PIDF 参数 (基于 12V 标准电压标定)。
     * @param kP 比例项系数
     * @param kI 积分项系数
     * @param kD 微分项系数
     * @param kF 前馈系数 (通常 ~= 1.0 / 理论最大速度)
     */
    public XKPIDFController setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
        return this;
    }

    public XKPIDFController setTargetVelocity(double targetVel) {
        this.targetVel = Math.max(0.0, targetVel);
        return this;
    }

    public XKPIDFController setBangBang(boolean enabled, double bandAbs, double assistPower) {
        this.bangBangEnabled = enabled;
        this.bandAbs = bandAbs;
        this.assistPower = assistPower;
        return this;
    }

    /**
     * 计算下一周期的电机功率。
     * @param currentVel 当前实测速度 (ticks/s)
     * @return Output 包含 power 值
     */
    @NonNull
    public Output update(double currentVel) {
        // 1. 计算 dt (使用纳秒提高精度)
        long currentNano = System.nanoTime();
        if (isFirstRun) {
            lastNanoTime = currentNano;
            isFirstRun = false;
            return new Output(0, false);
        }

        double seconds = (currentNano - lastNanoTime) / 1.0E9;
        lastNanoTime = currentNano;

        // 避免过长 dt 异常 (断点或极低频调用)
        if (seconds > 0.2) {
            seconds = 0.01;
        }

        // 2. Bang-Bang 判定 (最高优先级)
        if (bangBangEnabled && targetVel > 0 && currentVel < targetVel - bandAbs) {
            // 这里不重置积分，允许 PID 在后台维持(或根据策略选择清零)。
            // 简单起见，掉速时直接满力挽救。
            return new Output(assistPower, true);
        }

        // 3. 计算标准 PIDF
        double error = targetVel - currentVel;

        // 积分 (抗饱和)
        integral += error * seconds;
        if (kI != 0) {
            double maxI = integralLimit / Math.abs(kI);
            integral = Math.max(-maxI, Math.min(maxI, integral));
        }

        // 微分
        double derivative = (error - lastError) / seconds;
        lastError = error;

        // 基础输出 (at 12V)
        double output12V = (targetVel * kF) + (error * kP) + (integral * kI) + (derivative * kD);

        // 4. 电压补偿
        double currentVoltage = 12.0;
        try {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) currentVoltage = v;
        } catch (Exception ignored) {}

        // 如果当前电压只有 10V，需要给 12/10 = 1.2 倍的占空比才能达到同等效果
        double compensatedPower = output12V * (nominalVoltage / currentVoltage);

        // 5. 限幅
        double finalPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

        return new Output(finalPower, false);
    }

    /** 重置控制器状态 (如积分、微分历史) */
    public void reset() {
        integral = 0;
        lastError = 0;
        isFirstRun = true;
        // timer.reset(); // 不再使用 ElapsedTime
        lastNanoTime = System.nanoTime();
    }
}
