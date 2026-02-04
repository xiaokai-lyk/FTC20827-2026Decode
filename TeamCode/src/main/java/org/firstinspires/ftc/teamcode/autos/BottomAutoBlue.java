package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bottom Auto Blue", group = "Auto")
public class BottomAutoBlue extends BottomAutoBase {
    public BottomAutoBlue() {
        super(360.44, 151.95, 0, 144, 90,
                new Pose(59, 13.5, Math.toRadians(180)),
                new Pose(14, 10, Math.toRadians(180)),
                new Pose(40.4, 31.4, Math.toRadians(180))
        );
    }
}
