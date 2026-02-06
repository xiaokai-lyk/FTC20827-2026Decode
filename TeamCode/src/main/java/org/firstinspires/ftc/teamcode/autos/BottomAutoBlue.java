package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bottom Auto Blue", group = "Auto")
public class BottomAutoBlue extends BottomAutoBase {
    public BottomAutoBlue() {
        super(310, 120, 90,
                new Pose(59, 10, Math.toRadians(180)),
                new Pose(15, 10, Math.toRadians(180)),
                new Pose(15, 37, Math.toRadians(180)),
                new Pose(57, 43),
                new Pose(40.4, 31.4, Math.toRadians(180))
        );
    }
}
