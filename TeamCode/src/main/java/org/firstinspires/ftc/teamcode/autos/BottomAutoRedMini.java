package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bottom Auto Red Mini", group = "Auto")
public class BottomAutoRedMini extends BottomAutoBaseMini{
    public BottomAutoRedMini() {
        super(
                310,-120, -90,
                new Pose(85, 10, 0),
                new Pose(130, 10, 0),
                new Pose(130, 20, 0),
                new Pose(102.6, 32, 0)
        );
    }
}
