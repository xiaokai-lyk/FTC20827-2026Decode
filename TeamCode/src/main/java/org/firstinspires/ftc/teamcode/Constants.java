package org.firstinspires.ftc.teamcode;
public class Constants {
    public static class ShooterConfig {
        public int frontVelocity;
        public int backVelocity;
        public ShooterConfig(int frontVelocity, int backVelocity) {
            this.frontVelocity = frontVelocity;
            this.backVelocity = backVelocity;
        }
    }

    public static double preShooterStop = 0.6;
    public static double preShooterRun = -0.6;
    public static ShooterConfig shooterFar = new ShooterConfig(1400, 1400);
    public static ShooterConfig shooterNear = new ShooterConfig(700, 700);
    public static double intakePower = 0.7;
}
