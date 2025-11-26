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

    // 防止频繁写入：记录上次实际应用的 PID
    private PIDFCoefficients currentFrontApplied = null;
    private PIDFCoefficients currentBackApplied = null;


    // 新增：工作模式枚举与当前模式
    public enum ShooterMode { LOW, MID, HIGH }
    /**
     * -- GETTER --
     *  获取当前（实际或手动指定的）工作模式。
     */
    @Getter
    private ShooterMode currentMode = ShooterMode.MID;
    // 是否由系统自动选择模式；默认开启自动
    // private boolean autoMode = true;

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

    /**
     * 自适应 PID 更新：根据当前速度与目标速度之间的误差大小，在激进与保守 PID 之间切换或平滑插值。
     * 调用频率：建议在主循环 (opMode loop) 每帧调用。
     * @param mode 所需的工作模式。
     */
    public void setShooterMode(ShooterMode mode){
        this.currentMode = mode;

        PIDFCoefficients newFrontCoeffs = getCoeffsForMode(mode, true);
        PIDFCoefficients newBackCoeffs = getCoeffsForMode(mode, false);

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


    /**
     * 根据模式返回对应的 PIDFCoefficients（区分前/后电机）。
     */
    private PIDFCoefficients getCoeffsForMode(ShooterMode mode, boolean isFront){
        switch(mode){
            case LOW:
                return isFront ? Constants.ShooterPID.lowFrontPID : Constants.ShooterPID.lowBackPID;
            case HIGH:
                return isFront ? Constants.ShooterPID.highFrontPID : Constants.ShooterPID.highBackPID;
            case MID:
            default:
                return isFront ? Constants.ShooterPID.midFrontPID : Constants.ShooterPID.midBackPID;
        }
    }
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
