package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TopAutoRed", group = "Auto")
public class TopAutoRed extends TopAutoBase {
    public TopAutoRed() {
        super(
            30, 75, 0,

            // start pos
            new Pose(114, 130.5, Math.toRadians(90)),

            new Pose(97, 84.3, Math.toRadians(0)),
            new Pose(124, 84.3, Math.toRadians(0)),
            new Pose(97, 59.5, Math.toRadians(0)),
            new Pose(128, 59.5, Math.toRadians(0)),

            // shooting pos 1
            new Pose(104.5, 103, Math.toRadians(45)),

            // shooting pos 2
            new Pose(68, 68, Math.toRadians(45)),

            new Pose(117.4, 71, Math.toRadians(0)),
            new Pose(126, 73, Math.toRadians(0)),

            // end pos
            new Pose(96, 73, Math.toRadians(0)),

            new Pose(134.2, 58, Math.toRadians(50)),

            // last pose (no heading)
            new Pose(112.2, 40.9)
        );
    }
}
