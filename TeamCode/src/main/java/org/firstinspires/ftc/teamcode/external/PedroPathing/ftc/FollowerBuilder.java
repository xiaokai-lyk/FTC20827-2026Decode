package org.firstinspires.ftc.teamcode.external.PedroPathing.ftc;
import org.firstinspires.ftc.teamcode.external.PedroPathing.core.Drivetrain;
import org.firstinspires.ftc.teamcode.external.PedroPathing.core.follower.Follower;
import org.firstinspires.ftc.teamcode.external.PedroPathing.core.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.drivetrains.MecanumConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.constants.DriveEncoderConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.constants.OTOSConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.constants.PinpointConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.constants.ThreeWheelConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.constants.ThreeWheelIMUConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.constants.TwoWheelConstants;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.localizers.DriveEncoderLocalizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.localizers.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.ftc.localization.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.core.localization.Localizer;
import org.firstinspires.ftc.teamcode.external.PedroPathing.core.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** This is the FollowerBuilder.
 * It is used to create Followers with a specific drivetrain + localizer without having to use a full constructor
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class FollowerBuilder {
    private final FollowerConstants constants;
    private PathConstraints constraints;
    private final HardwareMap hardwareMap;
    private Localizer localizer;
    private Drivetrain drivetrain;

    public FollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        this.constants = constants;
        this.hardwareMap = hardwareMap;
        constraints = PathConstraints.defaultConstraints;
    }

    public FollowerBuilder setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    public FollowerBuilder driveEncoderLocalizer(DriveEncoderConstants lConstants) {
        return setLocalizer(new DriveEncoderLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder OTOSLocalizer(OTOSConstants lConstants) {
        return setLocalizer(new OTOSLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder pinpointLocalizer(PinpointConstants lConstants) {
        return setLocalizer(new PinpointLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder threeWheelIMULocalizer(ThreeWheelIMUConstants lConstants) {
        return setLocalizer(new ThreeWheelIMULocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder threeWheelLocalizer(ThreeWheelConstants lConstants) {
        return setLocalizer(new ThreeWheelLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder twoWheelLocalizer(TwoWheelConstants lConstants) {
        return setLocalizer(new TwoWheelLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }

    public FollowerBuilder mecanumDrivetrain(MecanumConstants mecanumConstants) {
        return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
    }

    public FollowerBuilder pathConstraints(PathConstraints pathConstraints) {
        this.constraints = pathConstraints;
        PathConstraints.setDefaultConstraints(pathConstraints);
        return this;
    }

    public Follower build() {
        return new Follower(constants, localizer, drivetrain, constraints);
    }
}
