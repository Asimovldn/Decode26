package org.firstinspires.ftc.teamcode.AutoOpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import static java.lang.Math.toDegrees;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOpModes.Kronos3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.jetbrains.annotations.Nullable;

@Autonomous(name = "Red Alliance Auto", preselectTeleOp = "Kronos3")
@Configurable
public class Kronos3Auto extends LinearOpMode {
    private Camera cam;
    private Shooter shooter;
    private Intake intake;
    private Follower follower;
    private int TARGET_TAG = 20;

    private Timer pathTimer, opModeTimer;

    public enum PathState {

    }

    private boolean activeCam, camOn;

    PID Lock;

    public final Pose shootPose = new Pose(120, 30, Math.toRadians(180));
    public final Pose startPose = new Pose(20, 120, Math.toRadians(0));

    private PathChain goToShoot, returnToShoot, pickUpA;

    enum Places {
        SHOOT,
        PICKUP_A,
        PICKUP_B,
        PICKUP_C,
        RETURN,
    }

    @Nullable
    private PathChain goTo(Places place) {
        if (place == Places.SHOOT) {
            return goToShoot;
        }
        if (place == Places.RETURN) {
            return returnToShoot;
        }

        if (place == Places.PICKUP_A) {
            return pickUpA;
        }

        return null;
    }

    public void buildPaths(Pose current) {

        goToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        current,
                        shootPose
                ))
                .setLinearHeadingInterpolation(
                        current.getHeading(),
                        shootPose.getHeading()
                )
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);

        cam = new Camera(hardwareMap).start();

        intake = new Intake(hardwareMap, follower);
        shooter = new Shooter(hardwareMap, follower, intake, false);

        activeCam = true; camOn = true;

        Pose start = fromCamera();
        if (start != null) {
            follower.setStartingPose(start);
            follower.setPose(start);
        }

        while (opModeInInit()) {
            if (gamepad1.leftBumperWasPressed()) {
                TARGET_TAG = (TARGET_TAG == 20) ? 24 : 20;
            }

            telemetry.addLine("Current Alliance: " + (TARGET_TAG == 20 ? "Blue" : "Red"));
            telemetry.addLine("Press LEFT BUMPER to Change");
            telemetry.update();
        }

        Lock = new PID(new PIDCoefficients(0.8, 0.01, 0.4));
        Lock.setRange(-15, 15);
        Lock.normalize();

        waitForStart();

        buildPaths(start);

        while (opModeIsActive()) {
            if (activeCam && !camOn) {
                cam.start();
                camOn = true;
            } else if (!activeCam && camOn) {
                cam.stop();
                camOn = false;
            }

            if (!follower.isBusy()) {
                follower.followPath(goToShoot);
            }

            while (!follower.atParametricEnd()) {
                follower.update();
            };

            double err = cam.getX(TARGET_TAG, -toDegrees(follower.getHeading()));

            while (Math.abs(err) < 0.1) {
                Lock.setError(err);
                follower.turn(-Lock.calculate());
                
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getHeading());

                telemetry.update();
            };
        }
    }

    @Nullable
    public Pose fromCamera() {
        Pose3D current = cam.getPose(TARGET_TAG, follower.getHeading());

        if (current == null) {
            return null;
        }

        return new Pose(current.getPosition().x, current.getPosition().y, current.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
