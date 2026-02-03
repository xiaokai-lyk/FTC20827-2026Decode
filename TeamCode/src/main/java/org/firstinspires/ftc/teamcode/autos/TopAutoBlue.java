package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TopAutoBlue",group = "Auto")
public class TopAutoBlue extends TopAutoBase{
    public TopAutoBlue(){
        super( 30,75,
            new Pose(33,135.5,90),
            new Pose(47,84.3,180),
            new Pose(15,84.3,180),
            new Pose(47,59.5,180),
            new Pose(16,59.5,180),
            new Pose(47,96,135),  // shooting pos
            new Pose(26.6,71,180),
            new Pose(15,71,180),
            new Pose(48,73,90)
        );
    }
    /*
    这几个数都不准明天会重新测
     */
}

