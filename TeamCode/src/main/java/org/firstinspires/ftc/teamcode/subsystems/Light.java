package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardwares;

public class Light {
    private final Servo led;
    public Light(@NonNull Hardwares hardwares){
        this.led = hardwares.servos.led;
    }
    public InstantCommand setLightColor(int r, int g, int b) {
        // clamp inputs
        r = clampRGBRange(r);
        g = clampRGBRange(g);
        b = clampRGBRange(b);

        // convert to [0,1]
        double rf = r / 255.0;
        double gf = g / 255.0;
        double bf = b / 255.0;

        // HSV conversion
        double max = Math.max(rf, Math.max(gf, bf));
        double min = Math.min(rf, Math.min(gf, bf));
        double delta = max - min;
        double s = (max == 0) ? 0 : (delta / max);
        double h; // degrees [0,360)
        if (delta == 0) {
            h = 0; // undefined hue for grayscale -> set 0
        } else if (max == rf) {
            h = 60 * (((gf - bf) / delta) % 6);
        } else if (max == gf) {
            h = 60 * (((bf - rf) / delta) + 2);
        } else {
            h = 60 * (((rf - gf) / delta) + 4);
        }
        if (h < 0) h += 360;

        // thresholds
        final double VALUE_OFF_THR = 0.05;   // 近似黑 -> Off
        final double SAT_WHITE_THR = 0.12;   // 饱和度很低视为白/灰
        final double VALUE_WHITE_THR = 0.6;  // 明亮且低饱和 -> 白

        if (max <= VALUE_OFF_THR) {
            return new InstantCommand(
                    () -> led.setPosition(0.0) // very dark -> Off
            );
        }

        if (s <= SAT_WHITE_THR && max >= VALUE_WHITE_THR) {
            return new InstantCommand(
                    () -> led.setPosition(1) // bright low-sat -> White
            );
        }

        // 图示中各颜色对应的归一化占空比（基于脉宽 500..2500us 映射到 0..1）
        double pwm = calculatePwm(h);

        // 最后再根据亮度做少量校正（可选）：更暗的颜色，使占空比稍微降低一些，以更接近视觉
        // 这里采用简单缩放：当 v < 0.5 时按 v/0.5 缩小到 0..1 区间，但保持不低于 0
        if (max < 0.5) {
            double scale = max / 0.5; // 0..1
            // 把 pwm 拉回到从 Off(0)到当前 pwm 的线段上，保留色相信息但考虑亮度
            pwm = pwm * scale;
        }

        // clamp to [0,1]
        if (pwm < 0) pwm = 0;
        if (pwm > 1) pwm = 1;

        double finalPwm = pwm;
        return new InstantCommand(
                () -> led.setPosition(finalPwm)
        );
    }

    private static double calculatePwm(double h) {
        final double PWM_RED = 0.277;   // red
        final double PWM_VIOLET = 0.722; // violet
        final double H_VIOLET_DEG = 270.0; // 把紫设为 270deg（近似图中紫）

        // 将 hue (0..360) 映射到 0..H_RANGE 的环上距离（从红开始顺时针到紫）
        double hueForMap;
        if (h <= H_VIOLET_DEG) {
            hueForMap = h; // 0..270
        } else {
            // 如果 hue 在 (270,360)，把它当作靠近红的一端环回：
            // map 271..359 -> 接近 0..(360-270)=90 之内的低值，
            // 使红区（0）和接近360的紫红附近平滑过渡到红色一端。
            hueForMap = h - 360.0; // 变为负数，后面加 H_RANGE 以得到小正数
            hueForMap += H_VIOLET_DEG;  // 例如 h=350 -> hueForMap = 350-360+270 = 260
            // 这样 0..360 都会被投射到大致 0..270 的区间
        }

        // 计算比例并线性插值到 PWM 区间
        double frac = hueForMap / H_VIOLET_DEG;
        if (frac < 0) frac = 0;
        if (frac > 1) frac = 1;
        return PWM_RED + frac * (PWM_VIOLET - PWM_RED);
    }

    private int clampRGBRange(int x) {
        if (x < 0) return 0;
        return Math.min(x, 255);
    }

}
