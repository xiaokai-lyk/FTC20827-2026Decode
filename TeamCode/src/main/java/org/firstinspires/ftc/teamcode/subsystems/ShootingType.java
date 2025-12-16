package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;

public class ShootingType {
    private enum Types {
        CLOSE,
        MIDDLE,
        FAR
    }

    private Types type;

    public ShootingType() {
        this.type = Types.CLOSE;
    }

    public InstantCommand setShootingType(int type) {
        return new InstantCommand(() -> {
            if (type == 0) {
                this.type = Types.CLOSE;
            } else if (type == 1) {
                this.type = Types.MIDDLE;
            } else {
                this.type = Types.FAR;
            }
        });
    }

    public boolean isClose() {
        return type == Types.CLOSE;
    }

    public boolean isMiddle() {
        return type == Types.MIDDLE;
    }

    public boolean isFar() {
        return type == Types.FAR;
    }
}
