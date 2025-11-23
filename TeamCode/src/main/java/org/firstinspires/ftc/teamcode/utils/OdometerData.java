package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import lombok.Getter;

/**
 * 数据容器：从外置里程计 / 定位计算机获取的机器人运动状态。
 * 所有量以机器人自身坐标系定义：x 向右为正，y 向前为正。
 */
public class OdometerData {
    // 当前航向（弧度，场地坐标系，右手系，通常来自 IMU / 外部融合）
    @Getter
    private final double headingRadians;
    // 机器人坐标系下的平移速度 (m/s 或 归一化，需与阻尼算法假设一致)
    @Getter
    private final double robotVx;
    @Getter
    private final double robotVy;
    // 可选：外部提供的瞬时航向角速度（rad/s），若不可用填 Double.NaN
    @Getter
    private final double yawRate;

    @Getter
    private final Pose2D robotPosition;

    public OdometerData(double headingRadians,
                        double robotVx,
                        double robotVy,
                        double yawRate,
                        Pose2D pos) {
        this.headingRadians = headingRadians;
        this.robotVx = robotVx;
        this.robotVy = robotVy;
        this.yawRate = yawRate;
        this.robotPosition = pos;
    }

    public OdometerData(GoBildaPinpointDriver odo){
        this.headingRadians = odo.getHeading(AngleUnit.RADIANS);
        this.robotVx = odo.getVelX(DistanceUnit.METER);
        this.robotVy = odo.getVelY(DistanceUnit.METER);
        this.yawRate = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        this.robotPosition = odo.getPosition();
    }
}

