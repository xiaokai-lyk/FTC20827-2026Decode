package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "TeleOp Blue Bottom", group = "TeleOp")
public class TeleOpBlueBottom extends TeleOpBase {
    public TeleOpBlueBottom() {
        super(293.39, 107.5, 90, new Pose2D(DistanceUnit.CM, 60, 230, AngleUnit.DEGREES, -90));
    }
}