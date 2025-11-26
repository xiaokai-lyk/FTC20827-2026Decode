package org.firstinspires.ftc.teamcode.utils;

import lombok.Getter;

/**
 * 通用 PID 控制器，所有参数由 Constants 统一管理。
 */
public class AdaptivePIDController {
    // Getters and setters for runtime tuning
    // make coefficients mutable so we can tune at runtime
    @Getter
    private double kp;
    @Getter
    private double ki;
    @Getter
    private double kd;
    private final double maxOutput;
    private final double minCommand;
    private final double deadzone;
    private final double maxIAccum;

    // state
    private double iAccum = 0.0;
    private double prevError = 0.0;
    private boolean first = true;

    public AdaptivePIDController(double kp, double ki, double kd,
                                 double maxOutput, double minCommand,
                                 double deadzone, double maxIAccum) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.maxOutput = Math.max(0.0, maxOutput);
        this.minCommand = Math.max(0.0, minCommand);
        this.deadzone = Math.max(0.0, deadzone);
        this.maxIAccum = Math.max(0.0, maxIAccum);
    }

    public void reset() {
        iAccum = 0.0;
        prevError = 0.0;
        first = true;
    }

    public double compute(double error, double dtSec) {
        if (Math.abs(error) <= deadzone) {
            prevError = error;
            first = false;
            return 0.0;
        }
        double p = kp * error;
        if (ki != 0.0 && dtSec > 1e-6) {
            iAccum += ki * error * dtSec;
            if (iAccum > maxIAccum) iAccum = maxIAccum;
            else if (iAccum < -maxIAccum) iAccum = -maxIAccum;
        }
        double d = 0.0;
        if (!first && kd != 0.0 && dtSec > 1e-6) {
            d = kd * (error - prevError) / dtSec;
        }
        double out = p + iAccum + d;
        if (out != 0.0 && Math.abs(out) < minCommand) {
            out = Math.signum(out) * minCommand;
        }
        if (out > maxOutput) out = maxOutput;
        else if (out < -maxOutput) out = -maxOutput;
        prevError = error;
        first = false;
        return out;
    }

    public AdaptivePIDController copy() {
        return new AdaptivePIDController(kp, ki, kd, maxOutput, minCommand, deadzone, maxIAccum);
    }

    public void setPID(double kp, double ki, double kd){ this.kp = kp; this.ki = ki; this.kd = kd; }
}
