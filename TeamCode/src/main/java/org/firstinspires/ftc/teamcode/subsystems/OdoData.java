package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * 数据容器：从外置里程计 / 定位计算机获取的机器人运动状态。
 * 所有量以机器人自身坐标系定义：x 向右为正，y 向前为正。
 */
public class OdoData {
    // 当前航向（弧度，场地坐标系，右手系，通常来自 IMU / 外部融合）
    private final double headingRadians;
    private final double headingDegrees;
    // 机器人坐标系下的平移速度 (m/s 或 归一化，需与阻尼算法假设一致)
    private final double robotX;
    private final double robotY;
    private final double robotVx;
    private final double robotVy;
    // 可选：外部提供的瞬时航向角速度（rad/s），若不可用填 Double.NaN
    private final double yawRate;
    private final Pose2D robotPosition;

    public OdoData(double headingRadians,
                   double headingDegrees,
                   double robotX,
                   double robotY,
                   double robotVx,
                   double robotVy,
                   double yawRate,
                   Pose2D pos) {
        this.headingRadians = headingRadians;
        this.headingDegrees = headingDegrees;
        this.robotX = robotX;
        this.robotY = robotY;
        this.robotVx = robotVx;
        this.robotVy = robotVy;
        this.yawRate = yawRate;
        this.robotPosition = pos;
    }

    public OdoData(@NonNull GoBildaPinpointDriver odo) {
        this.headingRadians = odo.getHeading(AngleUnit.RADIANS);
        this.headingDegrees = odo.getHeading(AngleUnit.DEGREES);
        this.robotX = odo.getPosX(DistanceUnit.CM);
        this.robotY = odo.getPosY(DistanceUnit.CM);
        this.robotVx = odo.getVelX(DistanceUnit.CM);
        this.robotVy = odo.getVelY(DistanceUnit.CM);
        this.yawRate = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        this.robotPosition = odo.getPosition();
    }

    public double getHeadingRadians() {
        return headingRadians;
    }

    public double getHeadingDegrees() {
        return headingDegrees;
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    public double getRobotVx() {
        return robotVx;
    }

    public double getRobotVy() {
        return robotVy;
    }

    public double getYawRate() {
        return yawRate;
    }

    public Pose2D getRobotPosition() {
        return robotPosition;
    }
}
