package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class XKCommandOpmode extends CommandOpMode {
    public abstract void onStart();
    public abstract void functionalButtons();
    @Override
    public void runOpMode() {
        initialize();

        functionalButtons();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        onStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }

}