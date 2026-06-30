package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.openftc.apriltag.AprilTagDetection;

public class TurretAlignTest extends LinearOpMode {
    private Limelight3A cam = hardwareMap.get(Limelight3A.class, "limelight");
    private Turret turret = new Turret();

    private GoBildaPinpointDriver pinpoint;

    private TelemetryManager telemetryM;

    private Follower follower;
    private Pose currentPose;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        cam.pipelineSwitch(0);
        cam.start();

        currentPose = new Pose();

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.update();

            updateTeleOp();

            telemetryM.debug("Position", follower.getPose().toString());
            telemetryM.debug("Velocity", follower.getVelocity().toString());
        }
    }

    public void updateTeleOp() {
        currentPose = follower.getPose();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
    }

    public void updateTurret() {}

}
