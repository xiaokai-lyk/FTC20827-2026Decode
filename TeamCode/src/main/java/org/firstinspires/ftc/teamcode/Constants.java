package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.AdaptivePIDController;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;

@Config
public class Constants {
    @Config
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

    public static class Position {
        public int x;
        public int y;
        public int heading;
        public Position(int x, int y, int heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

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
    public static int driveMaxVelocity = 2960;

    public static Position[] bluePickUpPositionTop = {  //x,y,heading
            new Position(-108,-70,90), //Blue Top Intake
            new Position(-166,-65,90), //Blue Middle Intake
            new Position(-228,-65,90) //Blue Bottom Intake,
    };
    public static Position[] bluePickUpPosition = {
            new Position(90,138,180),
            new Position(75,38,90)
    };


    public static Position[] blueBallPosition ={
            new Position(0,138,180),
            new Position(75, 155, 90)
    };

    public static double[] bluePreShootingPos={
            25.0,30.2,20.4
    };

    public static Position[] blueShootingPosTop = { //x,y,heading
            new Position(-52,-52,46), //Blue close
            new Position(-150,-150,45),  //Blue middle
    };
    public static double[][] blueShootingPosBottom = { //x,y,heading
            {7.5,16,18},   //Blue far
    };
    public static Position blueParkPositionTop = new Position(-140,-30,0); //x,y,heading
    public static Position blueParkPositionBottom = new Position(30,80,0); //x,y,heading
    public static Position blueGatePosition = new Position(-136,34,90);  // open gate
    public static Position blueGateControlPoint = new Position(-168, -30, 90);
    // 修改：redPickUpPosition → Position[]
    public static Position[] redPickUpPosition = {
            new Position(90, -138, -180),
            new Position(75, 0, -90)
    };

    // 修改：redBallPosition → Position[]
    public static Position[] redBallPosition = {
            new Position(0, -138, -180),
            new Position(75, -155, -90)
    };

    // 修改：redPickUpPositionTop → Position[]
    public static Position[] redPickUpPositionTop = {
            new Position(-108, 70, -90),
            new Position(-166, 65, -90),
            new Position(-228, 65, -90)
    };

    // 修改：redShootingPosTop → Position[]
    public static Position[] redShootingPosTop = {
            new Position(-52, 52, -46), //Red Close
            new Position(-150, 150, -45), //Red Middle
            // new Position()   //Red far (保留注释)
    };

    // 修改：redShootingPosBottom → Position[]
    public static Position[] redShootingPosBottom = {
            new Position(7, -16, -18),
    };

    // 修改：redParkPositionTop → Position
    public static Position redParkPositionTop = new Position(-140, 30, 0);

    // 修改：redParkPosition → Position
    public static Position redParkPosition = new Position(30, -80, 0);

    // 修改：redGatePosition → Position
    public static Position redGatePosition = new Position(-136, -34, -90);

    // 修改：redGateControlPoint → Position
    public static Position redGateControlPoint = new Position(-168, 30, -90);

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

    // ========= PID 可配置实例 =========
    public static class PID {
        // 平移 PID 参数（X/Y）
        public static final double TRANS_KP = 1.5;
        public static final double TRANS_KI = 0.00;
        public static final double TRANS_KD = 0.07;
        public static final double TRANS_MAX_OUT = 1.0;
        public static final double TRANS_MIN_CMD = 0.00;
        public static final double TRANS_DEADZONE_CM = 0.5;
        public static final double TRANS_I_CLAMP = 0.2;

        // 旋转 PID 参数
        public static final double ROT_KP = 1.0;
        public static final double ROT_KI = 0.00;
        public static final double ROT_KD = 0.05;
        public static final double ROT_MAX_OUT = 1.0;
        public static final double ROT_MIN_CMD = 0.00;
        public static final double ROT_DEADZONE_RAD = Math.toRadians(3.0);
        public static final double ROT_I_CLAMP = 0.2;

        // 统一通过常量初始化 PID 控制器
        private static final AdaptivePIDController XTRANS_BASE =
                new AdaptivePIDController(
                        TRANS_KP, TRANS_KI, TRANS_KD,
                        TRANS_MAX_OUT, TRANS_MIN_CMD,
                        TRANS_DEADZONE_CM, TRANS_I_CLAMP);
        private static final AdaptivePIDController YTRANS_BASE =
                new AdaptivePIDController(
                        TRANS_KP, TRANS_KI, TRANS_KD,
                        TRANS_MAX_OUT, TRANS_MIN_CMD,
                        TRANS_DEADZONE_CM, TRANS_I_CLAMP);
        private static final AdaptivePIDController ROTATE_BASE =
                new AdaptivePIDController(
                        ROT_KP, ROT_KI, ROT_KD,
                        ROT_MAX_OUT, ROT_MIN_CMD,
                        ROT_DEADZONE_RAD, ROT_I_CLAMP);

        public static AdaptivePIDController xTrans() { return XTRANS_BASE.copy(); }
        public static AdaptivePIDController yTrans() { return YTRANS_BASE.copy(); }
        public static AdaptivePIDController rotate() { return ROTATE_BASE.copy(); }

        // 便捷工厂：创建一个 AdaptivePoseController
        public static AdaptivePoseController newPoseController() {
            return new AdaptivePoseController(xTrans(), yTrans(), rotate());
        }
    }
}
