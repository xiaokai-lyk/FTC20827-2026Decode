package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TopAutoRed", group = "Auto")
public class TopAutoRed extends TopAutoBase {
    public TopAutoRed() {
        super(
            30, 75, 0,

            new Pose(114, 130.5, Math.toRadians(90)),

            new Pose(100.5, 100.5, Math.toRadians(47)),

            new Pose(97, 84.3, Math.toRadians(0)),
            new Pose(124, 84.3, Math.toRadians(0)),
            new Pose(97, 59.5, Math.toRadians(0)),
            new Pose(128, 59.5, Math.toRadians(0)),

            new Pose(129, 70.71, Math.toRadians(90)),
            new Pose(126, 73),

            // end pos
            new Pose(96, 73, Math.toRadians(0))
        );
    }
}
