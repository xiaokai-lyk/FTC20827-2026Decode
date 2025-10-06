package org.firstinspires.ftc.teamcode.subsystems.drive;

public class FeedforwardController {
    private final double mass; // kg
    private final double inertia; // kg*m^2
    private final double coeff; // 可调系数

    public FeedforwardController(double mass, double inertia, double coeff) {
        this.mass = mass;
        this.inertia = inertia;
        this.coeff = coeff;
    }

    public double[] calculate(double ax, double ay, double alpha, double L, double W) {
        double ff_x = mass * ax * coeff;
        double ff_y = mass * ay * coeff;
        double ff_t = inertia * alpha * coeff;
        return XKMecanumDrive.kinematicInverse(ff_x, ff_y, ff_t, L, W);
    }
}

