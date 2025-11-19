package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final DoubleSupplier x;
    private final DoubleSupplier rotate;
    private final DoubleSupplier y;
    private final DoubleSupplier powerCoefficient;
    private final MecanumDrive drive;
    private final DoubleSupplier heading;

    public TeleOpDriveCommand(
            MecanumDrive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotate,
            DoubleSupplier heading,
            DoubleSupplier powerCoefficient) {
        this.drive = drive;
        this.x = x;
        this.rotate = rotate;
        this.y = y;
        this.powerCoefficient = powerCoefficient;
        this.heading = heading;
    }

    @Override
    public void execute() {
        drive.driveFieldCentric(
                x.getAsDouble(),
                y.getAsDouble(),
                rotate.getAsDouble(),
                heading.getAsDouble(),
                powerCoefficient.getAsDouble()
        );
    }
}