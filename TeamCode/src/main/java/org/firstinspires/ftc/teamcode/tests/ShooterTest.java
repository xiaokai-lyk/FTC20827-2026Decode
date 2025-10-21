package org.firstinspires.ftc.teamcode.tests;

import static com.sun.tools.javac.tree.TreeInfo.name;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name="ShooterTest",group="tests")
@Config
public class ShooterTest extends LinearOpMode {
    public static double inputVelocity;
    private DcMotorEx LeftShooter;
    private DcMotorEx RightShooter;
    @Override
    public void runOpMode(){
        
    }


}
