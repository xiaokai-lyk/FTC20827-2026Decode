package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.drive.XK_TMecanumDrive;

@TeleOp(name = "XK_TDrive Test", group = "tests")
public class XK_TDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        XK_TMecanumDrive drive = new XK_TMecanumDrive(new Hardwares(hardwareMap), telemetry);
        drive.init();
        waitForStart();
        while (opModeIsActive()) {
            double strafe = gamepad1.left_stick_x; // Left stick X for strafing
            double forward = -gamepad1.left_stick_y; // Left stick Y for forward/backward
            double rotate = gamepad1.right_stick_x; // Right stick X for rotation

            drive.driveFieldCentric(strafe, forward, rotate);

            drive.addDataToTelemetry();
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Rotate", rotate);
            telemetry.update();
        }
    }
}
