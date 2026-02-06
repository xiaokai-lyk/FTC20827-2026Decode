package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bottom Auto Red", group = "Auto")
public class BottomAutoRed extends BottomAutoBase{
    public BottomAutoRed() {
        super(300, -120, -90,
                new Pose(85, 13.5, 0),
                new Pose(130, 10, 0),
                new Pose(103.6, 31.4, 0)
        );
    }
}
