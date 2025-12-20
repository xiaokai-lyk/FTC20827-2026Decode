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
    //我想要把最开始设置成45度这样子后面的坐标的heading是正的，但是这样子的话换边会出问题。虽然我们用绝对坐标换边肯定会出问题
    public static double[][] bluePickUpPositionTop = {  //x,y,heading
            {-108,-50,90}, //Blue Top Intake
            {-168,-45,90}, //Blue Middle Intake
            {-228,-45,90}, //Blue Bottom Intake,
    };
    public static double[][] bluePickUpPosition = {
            {90,138,180},
            {75,38,90}
    };


    public static double[][] blueBallPosition ={
            {0,138,180},
            {75, 155, 90}
    };
    public static double[][] blueShootingPosTop = { //x,y,heading
            {-55,-55,48}, //Blue close
            {-150,-150,45},  //Blue middle
            {}   //Blue far
    };
    public static double[][] blueShootingPosBottom = { //x,y,heading
            {7.5,16,18},   //Blue far
    };
    public static double[] blueParkPositionTop = {-140,-30,0}; //x,y,heading
    public static double[] blueParkPosition = {30,80,0}; //x,y,heading
    public static double[] blueGatePosition ={-125,42,90};  // open gate
    public static double[] blueGateControlPoint = {-168, -30, 90};

    public static double[][] redPickUpPositionTop = {  //x,y,heading
            {-108,50,-90}, //Red Top Intake (原Blue Top Intake的y变为正数且heading反转)
            {-168,45,-90}, //Red Middle Intake (同上)
            {-228,45,-90} //Red Bottom Intake (同上)
    };

    public static double[][] redShootingPosTop = { //x,y,heading
            {-55,55,-48}, //Red close (原blueShootingPosTop的y取反且heading反转)
            {-150,150,-45},  //Red middle (仅heading反转)
            {}   //Red far (未定义)
    };

    public static double[][] redShootingPosBottom = { //x,y,heading
            {-7.5,-16,-18},   //Red far (原blueShootingPosBottom的x和y取反)
    };

    public static double[] redParkPositionTop = {-140,30,0}; //x,y,heading (仅y变正)
    public static double[] redParkPosition = {-30,-80,0}; //x,y,heading (x变负)
    public static double[] redGatePosition ={-130,-42,-90};  // open gate (y变负)
    public static double[] redGateControlPoint = {-168, 30, -90}; // y变正


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
        public static final double TRANS_KP = 1.0;
        public static final double TRANS_KI = 0.00;
        public static final double TRANS_KD = 0.1;
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

