package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bottom Auto Blue", group = "Auto")
public class BottomAutoBlue extends BottomAutoBase {
    public BottomAutoBlue() {
        super(320, 122, 0, 144, 90,
                new Pose(55, 7, 180),
                new Pose(9, 7, 180),
                new Pose(44, 20, 180)
        );
    }
}
