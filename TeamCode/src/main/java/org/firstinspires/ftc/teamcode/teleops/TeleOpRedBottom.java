package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "TeleOp Red Bottom", group = "TeleOp")
public class TeleOpRedBottom extends TeleOpBase {
    public TeleOpRedBottom() {
        super(266.98, -97.83, -90, new Pose2D(DistanceUnit.CM, -84, 231, AngleUnit.DEGREES, 9110));
    }
}
