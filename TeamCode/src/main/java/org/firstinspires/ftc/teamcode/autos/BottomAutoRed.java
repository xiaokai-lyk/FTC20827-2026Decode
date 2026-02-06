package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bottom Auto Red", group = "Auto")
public class BottomAutoRed extends BottomAutoBase{
    public BottomAutoRed() {
        super(
                310,-120, -90,
                new Pose(85, 10, 0),
                new Pose(133, 10, 0),
                new Pose(133, 37, 0),
                new Pose(87, 43),
                new Pose(102.6, 32, 0)
        );
    }
}
