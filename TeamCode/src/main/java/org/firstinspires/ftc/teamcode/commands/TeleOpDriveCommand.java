package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final DoubleSupplier x;
    private final DoubleSupplier rotate;
    private final DoubleSupplier y;
    private final DoubleSupplier speedCoefficient;
    private final MecanumDrive drive;
    private final DoubleSupplier heading;
    private final BooleanSupplier useEncoders;

    public TeleOpDriveCommand(
            MecanumDrive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotate,
            DoubleSupplier heading,
            DoubleSupplier speedCoefficient,
            BooleanSupplier useEncoders
    ) {
        this.drive = drive;
        this.x = x;
        this.rotate = rotate;
        this.y = y;
        this.speedCoefficient = speedCoefficient;
        this.heading = heading;
        this.useEncoders = useEncoders;
    }

    @Override
    public void execute() {
        double[] potentials = drive.calculateComponents(
            x.getAsDouble(),
            y.getAsDouble(),
            rotate.getAsDouble(),
            heading.getAsDouble(),
            speedCoefficient.getAsDouble()
        );
        if(useEncoders.getAsBoolean()){
            drive.runWithEncoders();
            drive.setVelocity(potentials);
        } else {
            drive.runWithoutEncoders();
            drive.setPower(potentials);
        }

    }
}
