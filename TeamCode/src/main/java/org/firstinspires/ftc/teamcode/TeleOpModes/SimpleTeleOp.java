package org.firstinspires.ftc.teamcode.TeleOpModes;

import static org.firstinspires.ftc.teamcode.TeleOpModes.MovementTest.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Systems.ControlHandler;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

@TeleOp
public class SimpleTeleOp extends LinearOpMode  {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private ControlHandler gamepad1Control, gamepad2Control;
    private TelemetryManager telemetryM;
    private boolean robotCentric, triggerBased;

    @Override
    public void runOpMode() throws InterruptedException {
        robotCentric = true;
        triggerBased = false;

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap, follower);
        shooter = new Shooter(hardwareMap, follower, intake);

        follower.activateAllPIDFs();
        follower.setStartingPose(new Pose(72, 72));

        gamepad1Control = new ControlHandler(hardwareMap, gamepad1);
        gamepad2Control = new ControlHandler(hardwareMap, gamepad2);

        gamepad1Control.Add(ControlHandler.Buttons.A, this::switchCentric);
        gamepad2Control.Add(ControlHandler.Buttons.Y, this::switchShooterControl);

        while (opModeInInit()) {
            telemetryM.debug("Is Field Centric? ", robotCentric ? "False" : "True");
            telemetryM.debug("(press A to change)");
            telemetryM.debug("Method of Shooter Control: ", triggerBased ? "Trigger" : "Stick");
            telemetryM.debug("(press Y in Gamepad2 to change)");
            telemetryM.update(telemetry);
        }

        gamepad2Control.Add(ControlHandler.Buttons.X, shooter::switchMultiplier);
        gamepad2Control.Add(ControlHandler.Buttons.A, shooter::switchServo);

        waitForStart();

        follower.startTeleopDrive();
        follower.update();

        while (opModeIsActive()) {
            follower.updatePose();

            ArrayList<Double> s = new ArrayList<>();
            s.add(-90.0); s.add(0.0); s.add(90.0); s.add(180.0); s.add(-180.0);

            double heading = Math.toDegrees(follower.getHeading());

            double rot = 0;
            if (-gamepad1.right_stick_x < 0.1 && (heading - LazyMath.closest(heading, s) < 5)) {
                rot = 0.3 * Math.signum(heading - LazyMath.closest(heading, s));
            } else {
                rot = -gamepad1.right_stick_x;
            }

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, rot, robotCentric);
            follower.update();

            gamepad1Control.update();
            gamepad2Control.update();

            telemetryM.debug("Current Servo Position: ", shooter.getServoPosition());
            telemetryM.debug("Current Shooter Force: ", shooter.getMultiplier());
            telemetryM.debug("Current Shooter Speed: ", shooter.getVelocity());
            telemetryM.debug("Current Shooter PIDF: ", shooter.calculatePIDF());

            shooter.usingGamepad(gamepad2);
            intake.usingGamepad(gamepad2);

            telemetryM.update(telemetry);
        };
    }

    void switchCentric() {
        robotCentric = !robotCentric;
    }

    void switchShooterControl() {
        triggerBased = !triggerBased;
    }
}
