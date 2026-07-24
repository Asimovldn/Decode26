package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;

import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Systems.ControlHandler;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

import java.util.Locale;

@TeleOp
public class FestaJunina extends LinearOpMode {
    public Follower follower;
    public TelemetryManager telemetryM;
    public ControlHandler controller;
    public Shooter shooter;
    public Intake intake;
    public Mecanum drive;

    private final double tolerance = 15;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        follower.useHeading = true;
        follower.setStartingPose(new Pose(72, 72));
        follower.startTeleopDrive();
        follower.update();

        drive = (Mecanum) follower.drivetrain;

        intake = new Intake(hardwareMap, follower);
        shooter = new Shooter(hardwareMap, follower, intake);

        shooter.setMultiplier(1.2);

        Pose target = new Pose(72 + 48, 72);
        PID toAngle = new PID(new PIDCoefficients(1.7, 0.0183, 1.3), 0.00875);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        controller = new ControlHandler(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            controller.update();

            double dx = target.getX() - follower.getPose().getY();
            double dy = target.getY() - follower.getPose().getY();

            double angle = Math.atan2(dx, dy);
            double error = angle - follower.getHeading();

            error = normalize(error) / Math.PI; // normalizes into [-pi, pi] then into [-1, 1];

            if ((error * 360) < tolerance) {
                error *= 0.1;
            }

            toAngle.setError(error);

            double res = toAngle.calculate();

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;

            if (res >= 0.95 || res <= 0.95) {
                if (forward >= 0.5) {
                    forward *= 0.2;
                }
                if (strafe >= 0.5) {
                    strafe *= 0.2;
                }
            }

            telemetryM.debug(String.format("Error in Degrees: %.2f", error * 360));
            telemetryM.debug(String.format("PID Result in Degrees: %.2f", res * 360));
            telemetryM.debug("Current Pose: ", readPose(follower.getPose()));
            telemetryM.debug("Current Angle: ", Math.toDegrees(angle));
            telemetryM.debug("Difference in Distance: ", readPose(new Pose(dx, dy)));

            follower.setTeleOpDrive(forward, strafe, res);
            follower.update();

            shooter.usingTrigger(gamepad1);

            if (shooter.isBusy()) {
                intake.usingTrigger(gamepad1);
            }

            telemetryM.debug("TeleOp Drive Vectors: ", readVector(follower.getTeleopDriveVector()));
            telemetryM.update(telemetry);
            shooter.update();
            intake.update();
        }
    }

    public String readPose(Pose pose) {
        return String.format("X: %.2f, Y: %.2f", pose.getX(), pose.getY());
    }

    public String readVector(Vector vector) {
        return String.format("Magnitude: %.2f, Direction: %.2f", vector.getMagnitude(), vector.getTheta());
    }

    public double normalize(double angle) {
        while (angle > Math.PI) {
            angle -= Math.PI * 2;
        }

        while (angle < -Math.PI) {
            angle += Math.PI * 2;
        }

        return angle;
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.max(max, value));
    }
}
