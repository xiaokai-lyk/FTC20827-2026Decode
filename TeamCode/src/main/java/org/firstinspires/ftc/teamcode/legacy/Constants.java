package org.firstinspires.ftc.teamcode.legacy;

// 这个文件是前面版本的，里边的值仅作参考
public class Constants {
    public static class ShooterFarPIDF {
        public static double kP = 70;
        public static double kI = 3;
        public static double kD = 8;
        public static double kF = 0.25;
    }

    public static class ShooterConfig {
        public int frontVelocity;
        public int backVelocity;
        public ShooterConfig(int frontVelocity, int backVelocity) {
            this.frontVelocity = frontVelocity;
            this.backVelocity = backVelocity;
        }
    }

    // Position 类
    public static class Position {
        public double x;
        public double y;
        public double heading;
        public Position(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    //============Pan 数据==============

    // 云台电机参数
    public static final double panTicksPerDegree = 72.1;
    public static final double panMinAngleDegree = -180.0;
    public static final double panMaxAngleDegree = 180.0;
    public static final double panMaxPower = 0.85;

    // 锁定容差（度）
    public static final double PAN_LOCK_TOLERANCE_DEG = 5.0;

    // 视觉控制增益
    public static final double PAN_TX_KP = 0.5;
    // ====================== shooter and preshooter config ===============
    public static double preShooterBlock = -1;
    public static double preShooterRunClose = 1;
    public static double preShooterRunFar = 0.3;
    public static double preShooterRunMiddle = 0.9;
    public static ShooterConfig shooter40cm = new ShooterConfig(1100, 500);
    public static ShooterConfig shooter125cm = new ShooterConfig(500, 1200);
    public static ShooterConfig shooter250cm = new ShooterConfig(860, 1660);
    public static ShooterConfig shooterFar = new ShooterConfig(330, 2100);
    public static ShooterConfig shooterStop = new ShooterConfig(0, 0);
    public static double intakePowerClose = 1.0;
    public static double intakePowerOut = -1.0;
    public static int driveMaxVelocity = 2500; // 非常重要！！！！！！！

    // ===== Blue Positions (converted to Position[]) =====

    public static Position[] bluePickUpPositionTop = {
            new Position(-108, -50, 90),   // Blue Top Intake
            new Position(-168, -45, 90),   // Blue Middle Intake
            new Position(-228, -45, 90)    // Blue Bottom Intake
    };

    public static Position[] bluePickUpPosition = {
            new Position(90, 138, 180),
            new Position(75, 38, 90)
    };

    public static Position[] blueBallPosition = {
            new Position(0, 138, 180),
            new Position(75, 155, 90)
    };

    public static Position[] blueShootingPosTop = {
            new Position(-55, -55, 48),    // Blue close
            new Position(-150, -150, 45),  // Blue middle
            // Blue far is intentionally left out (empty in original)
    };

    public static Position[] blueShootingPosBottom = {
            new Position(7.5, 16, 18)      // Blue far
    };

    public static Position blueParkPositionTop = new Position(-140, -30, 0);
    public static Position blueParkPosition = new Position(30, 80, 0);
    public static Position blueGatePosition = new Position(-125, 42, 85);      // open gate
    public static Position blueGateControlPoint = new Position(-168, -30, 90);

    // ===== Red Positions (converted to Position[]) =====

    public static Position[] redBallPosition={};
    public static Position[] redPickUpPosition={};
    public static Position[] redPickUpPositionTop = {
            new Position(-108, 55, -90),   // Red Top Intake
            new Position(-168, 50, -90),   // Red Middle Intake
            new Position(-228, 50, -90)    // Red Bottom Intake
    };

    public static Position[] redShootingPosTop = {
            new Position(-55, 55, -48),    // Red close
            new Position(-150, 150, -45),  // Red middle
            // Red far is intentionally left out
    };

    public static Position[] redShootingPosBottom = {
            new Position(-7.5, -16, -18)   // Red far
    };

    public static Position redParkPositionTop = new Position(-140, 30, 0);
    public static Position redParkPosition = new Position(-30, -80, 0);
    public static Position redGatePosition = new Position(-130, -42, -85);     // open gate
    public static Position redGateControlPoint = new Position(-168, 30, -90);

//    // ===== Damping Constants =====
//    public static class Damping {
//        // 平移阻尼
//        public static final double TRANS_DEADZONE = 0.02;
//        public static final double TRANS_ALPHA = 0.35;
//        public static final double TRANS_NOISE = 0.02;
//        public static final double ADAPTIVE_K_MIN_T = 0.00;
//        public static final double ADAPTIVE_K_MAX_T = 0.50;
//        public static final double ADAPTIVE_K_SPEED_MAX_T = 0.30;
//        public static final int DEADZONE_RAMP_FRAMES_T = 6;
//        public static final double DEADZONE_RAMP_MIN_T = 0.50;
//        public static final double HIGH_DRIFT_THRESHOLD = 0.35;
//        public static final double DRIFT_MULTIPLIER = 2.0;
//        public static final double ROTATION_COUPLE_THRESHOLD = 0.6; // rad/s
//        public static final double ROTATION_COUPLE_FACTOR = 0.80;
//
//        // 角速度阻尼
//        public static final double ROTATE_DEADZONE = 0.02;
//        public static final double YAW_RATE_ALPHA = 0.30;
//        public static final double YAW_RATE_NOISE = 0.02; // rad/s
//        public static final double MAX_ROTATE_CMD = 1.0;
//        public static final double ADAPTIVE_K_MIN = 0.00;
//        public static final double ADAPTIVE_K_MAX = 0.25;
//        public static final double ADAPTIVE_K_SPEED_MAX = 0.22;
//        public static final int DEADZONE_RAMP_FRAMES = 6;
//        public static final double DEADZONE_RAMP_MIN = 0.30;
//        public static final double HIGH_RATE_THRESHOLD = 0.35; // rad/s
//        public static final double HIGH_RATE_MULTIPLIER = 1.25;
//    }
//
//    // ===== PID Configuration =====
//    public static class PID {
//        // 平移 PID 参数（X/Y）
//        public static final double TRANS_KP = 1.0;
//        public static final double TRANS_KI = 0.00;
//        public static final double TRANS_KD = 0.1;
//        public static final double TRANS_MAX_OUT = 1.0;
//        public static final double TRANS_MIN_CMD = 0.00;
//        public static final double TRANS_DEADZONE_CM = 0.5;
//        public static final double TRANS_I_CLAMP = 0.2;
//
//        // 旋转 PID 参数
//        public static final double ROT_KP = 1.0;
//        public static final double ROT_KI = 0.00;
//        public static final double ROT_KD = 0.05;
//        public static final double ROT_MAX_OUT = 1.0;
//        public static final double ROT_MIN_CMD = 0.00;
//        public static final double ROT_DEADZONE_RAD = Math.toRadians(3.0);
//        public static final double ROT_I_CLAMP = 0.2;
//
//        private static final AdaptivePIDController XTRANS_BASE =
//                new AdaptivePIDController(
//                        TRANS_KP, TRANS_KI, TRANS_KD,
//                        TRANS_MAX_OUT, TRANS_MIN_CMD,
//                        TRANS_DEADZONE_CM, TRANS_I_CLAMP);
//        private static final AdaptivePIDController YTRANS_BASE =
//                new AdaptivePIDController(
//                        TRANS_KP, TRANS_KI, TRANS_KD,
//                        TRANS_MAX_OUT, TRANS_MIN_CMD,
//                        TRANS_DEADZONE_CM, TRANS_I_CLAMP);
//        private static final AdaptivePIDController ROTATE_BASE =
//                new AdaptivePIDController(
//                        ROT_KP, ROT_KI, ROT_KD,
//                        ROT_MAX_OUT, ROT_MIN_CMD,
//                        ROT_DEADZONE_RAD, ROT_I_CLAMP);
//
//        public static AdaptivePIDController xTrans() { return XTRANS_BASE.copy(); }
//        public static AdaptivePIDController yTrans() { return YTRANS_BASE.copy(); }
//        public static AdaptivePIDController rotate() { return ROTATE_BASE.copy(); }
//
//        public static AdaptivePoseController newPoseController() {
//            return new AdaptivePoseController(xTrans(), yTrans(), rotate());
//        }
//    }
}
