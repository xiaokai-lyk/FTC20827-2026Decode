package org.firstinspires.ftc.teamcode.external.PedroPathing.core.control;

public interface Controller {
    double run();
    void reset();
    void updateError(double error);
    void updatePosition(double position);
}
