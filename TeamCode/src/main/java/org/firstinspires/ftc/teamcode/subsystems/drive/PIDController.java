package org.firstinspires.ftc.teamcode.subsystems.drive;

public class PIDController {
    private double kp, ki, kd;
    private double integral = 0;
    private double lastError = 0;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double error) {
        double p = kp * error;
        integral += error;
        double i = ki * integral;
        double d = kd * (error - lastError);
        lastError = error;
        return p + i + d;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}

