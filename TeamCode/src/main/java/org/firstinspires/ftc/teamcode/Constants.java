package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;

public class Constants {
    public static class ShooterConfig {
        public int frontVelocity;
        public int backVelocity;
        public ShooterConfig(int frontVelocity, int backVelocity) {
            this.frontVelocity = frontVelocity;
            this.backVelocity = backVelocity;
        }
    }

    public static double preShooterBlock = -1;
    public static double preShooterRun = 1;
    public static ShooterConfig shooter250cm = new ShooterConfig(860, 1660);
    public static ShooterConfig shooter150cm = new ShooterConfig(500, 1440);
    public static ShooterConfig shooter105cm = new ShooterConfig(500, 1300);
    public static ShooterConfig shooterStop = new ShooterConfig(0, 0);
    public static double intakePowerSlow = 0.8;
    public static double intakePowerFast = 1.0;
    public static int driveMaxVelocity = 2960;
    //我想要把最开始设置成45度这样子后面的坐标的heading是正的，但是这样子的话换边会出问题。虽然我们用绝对坐标换边肯定会出问题
    public static double[][] pickUpPosition = {  //x,y,heading
        {-108,-70,90},
        {-168,-70,90},
        {-228,-70,90}
    };
    // 吸球时把y轴怼到0就能吸进去。
    public static double[][] shootingPosition = { //x,y,heading
        {-85,-85,45}, //close
        {},           //middle
        {}            //far
    };


    // 新增：自适应阻尼相关常量集中管理
    public static class Damping {
        // 平移阻尼
        public static final double TRANS_DEADZONE = 0.02;
        public static final double TRANS_ALPHA = 0.35;
        public static final double TRANS_NOISE = 0.02;
        public static final double ADAPTIVE_K_MIN_T = 0.00;
        public static final double ADAPTIVE_K_MAX_T = 0.50;
        public static final double ADAPTIVE_K_SPEED_MAX_T = 0.30;
        public static final int DEADZONE_RAMP_FRAMES_T = 6;
        public static final double DEADZONE_RAMP_MIN_T = 0.50;
        public static final double HIGH_DRIFT_THRESHOLD = 0.35;
        public static final double DRIFT_MULTIPLIER = 2.0;
        public static final double ROTATION_COUPLE_THRESHOLD = 0.6; // rad/s
        public static final double ROTATION_COUPLE_FACTOR = 0.80;

        // 角速度阻尼
        public static final double ROTATE_DEADZONE = 0.02;
        public static final double YAW_RATE_ALPHA = 0.30;
        public static final double YAW_RATE_NOISE = 0.02; // rad/s
        public static final double MAX_ROTATE_CMD = 1.0;
        public static final double ADAPTIVE_K_MIN = 0.00;
        public static final double ADAPTIVE_K_MAX = 0.25;
        public static final double ADAPTIVE_K_SPEED_MAX = 0.22;
        public static final int DEADZONE_RAMP_FRAMES = 6;
        public static final double DEADZONE_RAMP_MIN = 0.30;
        public static final double HIGH_RATE_THRESHOLD = 0.35; // rad/s
        public static final double HIGH_RATE_MULTIPLIER = 1.25;
    }
}
