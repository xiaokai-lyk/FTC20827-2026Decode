package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;

/**
 * AutoDrive subsystem.
 * Convention:
 *  xCmd = forward/back (forward +)
 *  yCmd = left/right   (left +)
 *  rotateCmd = CCW +
 * When calling Drive.calculateComponents(x,y,rotate,...), its first param expects lateral (right+ normally).
 * We provide left+ so we pass yCmd directly; if your hardware expects right+ invert sign externally.
 */
public class AutoDrive {

    public static class Output {
        public final double xCmd; // forward +
        public final double yCmd; // left +
        public final double rotateCmd; // CCW +
        public final double dxCm; public final double dyCm; public final double dThetaRad;
        public final boolean atPosition; public final boolean atHeading;
        public Output(double xCmd, double yCmd, double rotateCmd,
                      double dxCm, double dyCm, double dThetaRad,
                      boolean atPosition, boolean atHeading){
            this.xCmd = xCmd; this.yCmd = yCmd; this.rotateCmd = rotateCmd;
            this.dxCm = dxCm; this.dyCm = dyCm; this.dThetaRad = dThetaRad;
            this.atPosition = atPosition; this.atHeading = atHeading;
        }
    }


    public Output driveToAdaptive(@NonNull Drive drive, @NonNull AdaptivePoseController controller,
                                  double targetXcm, double targetYcm, double targetHeadingRad,
                                  @NonNull OdometerData data, double speedCoefficient, boolean autoDecelerate){
        AdaptivePoseController.Output ao = controller.computeAdaptive(targetXcm, targetYcm, Math.toRadians(targetHeadingRad), data);
        // 动态减速: 剩余距离 < 10cm 时按比例降低速度系数 (保持最低20%)
        double distanceErr = Math.hypot(ao.dxCm, ao.dyCm);
        double scaledSpeed = speedCoefficient;
        if (distanceErr < 10.0 && autoDecelerate) {
            double ratio = distanceErr / 10.0; // 0..1
            scaledSpeed = speedCoefficient * Math.max(ratio, 0.2); // 最低保持 20% 以避免停滞
        }
        // 我不知道为什么，但是就是要把所有符号都弄成相反的才行！
        // 我觉得是ChatGPT的问题！如果有后人解决这个问题希望可以联系 kai_xk@qq.com
        // 这是这段代码里最不优雅的地方了！
        double[] wheel = drive.calculateComponents(-ao.xCmd, -ao.yCmd, -ao.rotateCmd, data.getHeadingRadians(), scaledSpeed);
        drive.setVelocity(wheel);
        return new Output(ao.xCmd, ao.yCmd, ao.rotateCmd, ao.dxCm, ao.dyCm, ao.dThetaRad, ao.atPosition, ao.atHeading);
    }
}
