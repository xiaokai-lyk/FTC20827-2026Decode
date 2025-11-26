package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.jetbrains.annotations.Contract;

import lombok.Getter;

public class Shooter {
    private final DcMotorEx shooterFront, shooterBack, preShooter, intake;

    private double shooterFrontTarget = 0;
    private double shooterBackTarget = 0;

    // 自适应 PID: 保存两套已经通过 ZN 整定得到的参数
    private PIDFCoefficients aggressiveFrontPID = new PIDFCoefficients(7, 68.767, 0.267, 7e-4);
    private PIDFCoefficients conservativeFrontPID = new PIDFCoefficients(3.3, 25.935, 0.277, 0.000);
    private PIDFCoefficients aggressiveBackPID = new PIDFCoefficients(7, 27.023, 0.705, 7e-4);
    private PIDFCoefficients conservativeBackPID = new PIDFCoefficients(3.3, 10.192, 0.680, 0.000);

    // 误差阈值（ticks/s），可根据测试调节：大误差 -> 激进；小误差 -> 保守；中间 -> 插值
    private int highErrorThreshold = 350; // 离目标较远
    private int lowErrorThreshold = 50;  // 接近目标

    // 防止频繁写入：记录上次实际应用的 PID
    private PIDFCoefficients currentFrontApplied = null;
    private PIDFCoefficients currentBackApplied = null;


    public Shooter(@NonNull Hardwares hardwares) {
        this.shooterFront = hardwares.motors.shooterFront;
        this.shooterBack = hardwares.motors.shooterBack;
        this.preShooter = hardwares.motors.preShooter;
        this.intake = hardwares.motors.intake;
    }

    @NonNull
    @Contract("_ -> new")
    public InstantCommand setShooter(Constants.ShooterConfig config) {
        return new InstantCommand(() -> {
            shooterFront.setVelocity(config.frontVelocity);
            shooterBack.setVelocity(config.backVelocity);
            shooterFrontTarget = config.frontVelocity;
            shooterBackTarget = config.backVelocity;

        });
    }

    public InstantCommand allowBallPass(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterRun)
        );
    }

    public InstantCommand blockBallPass(){
        return new InstantCommand(
                ()->preShooter.setPower(Constants.preShooterBlock)
        );
    }

    public InstantCommand stopPreShooter(){
        return new InstantCommand(
            ()->preShooter.setPower(0)
        );
    }

    public void updateMotorPIDF(Constants.ShooterConfig targetVelocity){
        shooterFrontTarget = targetVelocity.frontVelocity; // 若外部调用动态改变目标
        shooterBackTarget = targetVelocity.backVelocity;

        // 获取当前速度
        double frontVel = shooterFront.getVelocity();
        double backVel = shooterBack.getVelocity();

        double frontError = shooterFrontTarget - frontVel;
        double backError = shooterBackTarget - backVel;

        // 绝对误差用于判定区间
        double frontAbs = Math.abs(frontError);
        double backAbs = Math.abs(backError);

        PIDFCoefficients newFrontCoeffs = selectOrBlend(frontAbs, aggressiveFrontPID, conservativeFrontPID);
        PIDFCoefficients newBackCoeffs = selectOrBlend(backAbs, aggressiveBackPID, conservativeBackPID);

        // 若与当前已应用不同，则更新电机 PIDF
        if (!pidEquals(currentFrontApplied, newFrontCoeffs)) {
            shooterFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, newFrontCoeffs);
            currentFrontApplied = newFrontCoeffs;
        }
        if (!pidEquals(currentBackApplied, newBackCoeffs)) {
            shooterBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, newBackCoeffs);
            currentBackApplied = newBackCoeffs;
        }
    }

    private PIDFCoefficients selectOrBlend(double absError, PIDFCoefficients aggressive, PIDFCoefficients conservative){
        if (absError >= highErrorThreshold) return aggressive;
        if (absError <= lowErrorThreshold) return conservative;
        double t = (absError - lowErrorThreshold) / (double)(highErrorThreshold - lowErrorThreshold); // 0~1
        // 可使用平滑函数提高过渡自然度：t = t*t*(3-2*t); // smoothstep
        t = t * t * (3 - 2 * t);
        return blendPID(conservative, aggressive, t);
    }

    private PIDFCoefficients blendPID(PIDFCoefficients a, PIDFCoefficients b, double t) {
        double p = lerp(a.p, b.p, t);
        double i = lerp(a.i, b.i, t);
        double d = lerp(a.d, b.d, t);
        double f = lerp(a.f, b.f, t); // 多数情况下 F 不需要插值，可直接用保守或激进之一，这里保留插值灵活性
        return new PIDFCoefficients(p, i, d, f);
    }

    private double lerp(double x, double y, double t){ return x + (y - x) * t; }

    /**
     * 比较两个 PIDFCoefficients 是否相等（允许微小浮点差异）。
     */
    private boolean pidEquals(PIDFCoefficients a, PIDFCoefficients b){
        if (a == b) return true;
        if (a == null || b == null) return false;
        // 允许微小浮点差异，使用一个很小的 epsilon
        double eps = 1e-6;
        return Math.abs(a.p - b.p) < eps && Math.abs(a.i - b.i) < eps && Math.abs(a.d - b.d) < eps && Math.abs(a.f - b.f) < eps;
    }

}
