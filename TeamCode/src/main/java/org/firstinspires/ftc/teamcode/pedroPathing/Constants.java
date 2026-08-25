package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(1.5)
            .lateralZeroPowerAcceleration(-63.3132)
            .forwardZeroPowerAcceleration(-63.8586)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.075, 0.2, 0.075))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.017, 0.0015, 0.00025, 0.6, 0.001))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0.0005, 0.02, 0.013))
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.0073923, 0.002523871));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.4, 0.4);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.98)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(57.189)
            .yVelocity(27.1771);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .forwardPodY(4.66577)
            .strafePodX(-6.624593)
            .distanceUnit(DistanceUnit.INCH);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
