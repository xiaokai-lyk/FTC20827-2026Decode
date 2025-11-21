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
    public static ShooterConfig shooterFar = new ShooterConfig(860, 1660);
    public static ShooterConfig shooterMedium = new ShooterConfig(500, 1440);
    public static ShooterConfig shooterNear = new ShooterConfig(500, 1300);
    public static ShooterConfig shooterStop = new ShooterConfig(0, 0);
    public static double intakePower = 0.7;
    public static int driveMaxVelocity = 1500;
}
