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
            new Pose(22, 84.3, Math.toRadians(180)),
            new Pose(47, 59, Math.toRadians(180)),
            new Pose(18, 59, Math.toRadians(180)),
            new Pose(43, 99, Math.toRadians(130)),  // shooting pos1
            new Pose(65, 78, Math.toRadians(130)), // shooting pos 2
            new Pose(14.9, 74, Math.toRadians(90)), // gate
            new Pose(30,78),                   // gate control
            new Pose(30, 73, Math.toRadians(180))  // end pos
        );
    }
}
/* 拿侧板撞intake

 */
