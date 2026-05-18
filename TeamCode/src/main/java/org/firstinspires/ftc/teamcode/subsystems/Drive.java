package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


/** 
 * 驱动系统
 * 
 * 功能：
 * 1. 控制四个驱动轮电机
 * 2. 计算并设置机器人运动组件
 * 3. 提供基于编码器的速度控制和基于功率的直接控制
 */
public class Drive {
    
    // ==========================================
    // 常量配置
    // ==========================================
    private static final double MAX_VELOCITY = 2900.0;
    // 机器人运动学补偿因子
    private static final double TRANSLATION_COMPENSATION_FACTOR = 1.1;

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx mLeftFront, mRightFront, mLeftRear, mRightRear;

    /**
     * 构造驱动系统实例
     * @param hardwares 硬件映射
     */
    public Drive(@NonNull Hardwares hardwares){
        this.mLeftFront = hardwares.motors.mLeftFront;
        this.mRightFront = hardwares.motors.mRightFront;
        this.mLeftRear = hardwares.motors.mLeftRear;
        this.mRightRear = hardwares.motors.mRightRear;
        this.init();
    }

    private void init() {
        mLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mLeftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mRightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        mLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mLeftRear.setDirection(DcMotorEx.Direction.REVERSE);
        mRightRear.setDirection(DcMotorEx.Direction.FORWARD);

        mLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * 设置四个电机的功率
     * @param potentials 包含四个电机功率值的数组 [左前, 右前, 左后, 右后]
     */
    public void setPower(@NonNull double[] potentials){
        mLeftFront.setPower(potentials[0]);
        mRightFront.setPower(potentials[1]);
        mLeftRear.setPower(potentials[2]);
        mRightRear.setPower(potentials[3]);
    }

    /**
     * 设置四个电机的速度
     * @param potentials 包含四个电机速度值百分比的数组 [左前, 右前, 左后, 右后] (每个值的范围：[-1.0, 1.0])
     */
    public void setVelocity(@NonNull double[] potentials){
        mLeftFront.setVelocity(potentials[0] * MAX_VELOCITY);
        mRightFront.setVelocity(potentials[1] * MAX_VELOCITY);
        mLeftRear.setVelocity(potentials[2] * MAX_VELOCITY);
        mRightRear.setVelocity(potentials[3] * MAX_VELOCITY);
    }

    /**
     * 计算四个驱动轮的运动分量
     * @param x X轴方向输入 (-1.0 到 1.0)
     * @param y Y轴方向输入 (-1.0 到 1.0)
     * @param rotate 旋转输入 (-1.0 到 1.0)
     * @param headingRadians 当前机器人的朝向弧度
     * @param speedCoefficient 速度系数
     * @return 包含四个电机功率值的数组 [左前, 右前, 左后, 右后]
     */
    public double[] calculateComponents(double x, double y, double rotate, double headingRadians, double speedCoefficient){
        // 坐标变换：将场地方向转换为机器人方向
        double rotX = x * Math.cos(-headingRadians) - y * Math.sin(-headingRadians);
        double rotY = x * Math.sin(-headingRadians) + y * Math.cos(-headingRadians);
        // 应用平移补偿因子
        rotX = rotX * TRANSLATION_COMPENSATION_FACTOR;

        // 计算分母以归一化功率值
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        // 计算每个轮子的功率
        double frontLeftPower = (rotY + rotX + rotate) / denominator * speedCoefficient;
        double backLeftPower = (rotY - rotX + rotate) / denominator * speedCoefficient;
        double frontRightPower = (rotY - rotX - rotate) / denominator * speedCoefficient;
        double backRightPower = (rotY + rotX - rotate) / denominator * speedCoefficient;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }

    /**
     * 获取四个电机的当前速度
     * @return 包含四个电机速度值的数组 [左前, 右前, 左后, 右后]
     */
    public double[] getVelocities(){
        return new double[]{
                mLeftFront.getVelocity(),
                mRightFront.getVelocity(),
                mLeftRear.getVelocity(),
                mRightRear.getVelocity()
        };
    }

    public static class DriveCommand extends CommandBase {
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier rotateSupplier;
        private final DoubleSupplier ySupplier;
        private final double speedCoefficient;
        private final Drive drive;
        private final Supplier<PinpointDriverData> odometerDataSupplier;
        public final boolean useEncoders;
        private final boolean fieldCentric;
        private final double headingOffset;

        public double dampedX, dampedY, dampedRotate;

        /**
         * 构造驱动命令
         * @param drive 驱动系统实例
         * @param xSupplier X轴输入提供者
         * @param ySupplier Y轴输入提供者
         * @param rotateSupplier 旋转输入提供者
         * @param odometerData 里程计数据提供者
         * @param speedCoefficient 速度系数
         * @param useEncoders 是否使用编码器控制
         * @param fieldCentric 是否使用场地方位控制
         */
        public DriveCommand(
                Drive drive,
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                DoubleSupplier rotateSupplier,
                Supplier<PinpointDriverData> odometerData,
                double speedCoefficient,
                boolean useEncoders,
                boolean fieldCentric) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.rotateSupplier = rotateSupplier;
            this.odometerDataSupplier = odometerData;
            this.speedCoefficient = speedCoefficient;
            this.useEncoders = useEncoders;
            this.fieldCentric = fieldCentric;
            this.headingOffset = 0.0;
        }

        public DriveCommand(
                Drive drive,
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                DoubleSupplier rotateSupplier,
                Supplier<PinpointDriverData> odometerData,
                double speedCoefficient,
                boolean useEncoders,
                boolean fieldCentric,
                double headingOffset) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.rotateSupplier = rotateSupplier;
            this.odometerDataSupplier = odometerData;
            this.speedCoefficient = speedCoefficient;
            this.useEncoders = useEncoders;
            this.fieldCentric = fieldCentric;
            this.headingOffset = headingOffset;
        }

        @Override
        public void execute() {
            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double rotate = rotateSupplier.getAsDouble();
            PinpointDriverData odometerData = odometerDataSupplier.get();
            
            double currentHeading;
            if(fieldCentric)
            {
                currentHeading = odometerData.getHeadingRadians() + Math.toDegrees(headingOffset);
            }else{
                currentHeading = 0.0;
            }

            double[] potentials = drive.calculateComponents(
                    x,
                    y,
                    -rotate,
                    currentHeading,
                    speedCoefficient
            );

            if (useEncoders) {
                drive.setVelocity(potentials);
            } else {
                drive.setPower(potentials);
            }
        }
    }
}