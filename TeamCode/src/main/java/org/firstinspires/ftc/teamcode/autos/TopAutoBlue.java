package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TopAutoBlue", group = "Auto")
public class TopAutoBlue extends TopAutoBase {
    public TopAutoBlue() {
        super(
            30, 75,0,
            new Pose(30, 130.5, Math.toRadians(90)), // start pos
            new Pose(47, 84.3, Math.toRadians(180)),
            new Pose(20, 84.3, Math.toRadians(180)),
            new Pose(47, 61, Math.toRadians(180)),
            new Pose(16, 61, Math.toRadians(180)),
            new Pose(43, 99, Math.toRadians(130)),  // shooting pos1
            new Pose(68, 68, Math.toRadians(130)), // shooting pos 2
            new Pose(26.6, 73, Math.toRadians(180)),
            new Pose(18, 73, Math.toRadians(180)),
            new Pose(30, 73, Math.toRadians(180)),  // end pos
            new Pose(7.5,57, 2.39),
            new Pose(31.8,40.9)
        );
    }
}
