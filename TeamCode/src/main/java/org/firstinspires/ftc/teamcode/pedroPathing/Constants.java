package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.16) // in kg.
            .forwardZeroPowerAcceleration(-41.178829288419635)
            .lateralZeroPowerAcceleration(-80.89164717482416)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0,0.025, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(-1.4, 0, -0.05, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.0, 0.00005, 0.6, 0.01))
            .centripetalScaling(0.0005)
            ;

    // turn error threshold is 0.01

    // TODO HEADING PIDF (NEED HELP)
    // TODO DRIVE TUNING


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .xVelocity(55.243236631859)
            .yVelocity(40.622824120709275);

    public static PinpointConstants localizerConstants = new PinpointConstants() // TODO
            .forwardPodY(-134.112) // Values to change (negative or not)
            .strafePodX(23.0124) // Values to change (negative or not)
            // OLD VALUES BELOW
            /*.forwardPodY(-136.4) // Values to change
            .strafePodX(-22.1) // Values to change*/
            .distanceUnit(DistanceUnit.MM) // we use MM
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD) // Or use SWINGARM_POD
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1.35, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
