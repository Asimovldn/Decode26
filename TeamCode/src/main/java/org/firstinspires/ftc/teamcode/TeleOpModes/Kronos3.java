package org.firstinspires.ftc.teamcode.TeleOpModes;

import static java.lang.Math.toDegrees;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.ControlHandler;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.Nullable;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp
@Configurable
public class Kronos3 extends LinearOpMode {
    DcMotorEx blockMotor;
    Follower follower;
    Camera cam;
    boolean useCam = false;
    boolean aligning;

    private boolean isRobotCentric = true, isRedAlliance = true, isCameraOn = true;

    private BlockStates block;

    ElapsedTime clock;

    public int TARGET_TAG;

    ControlHandler controlA, controlB;

    PID Lock;

    TelemetryManager tele;
    Shooter shooter;
    Intake intake;

    public double forward, strafe, turn, last, flag;



    private ArrayList<Double> velocities;

    public enum BlockStates {
        BLOCKING,
        DEBOUCING,
        ZERO
    }


    @Override
    public void runOpMode() {
        blockMotor = hardwareMap.get(DcMotorEx.class, "storage");
        blockMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        controlA = new ControlHandler(hardwareMap, gamepad1);
        controlB = new ControlHandler(hardwareMap, gamepad2);

        if (hardwareMap.tryGet(Limelight3A.class, "limelight") == null) {
            useCam = false;
        } else {
            useCam = true;
        }

        cam = new Camera(hardwareMap);

        Lock = new PID(new PIDCoefficients(0.4, 0.009, 0.1));
        Lock.setRange(-15, 15);

        velocities = new ArrayList<>();

        follower.setStartingPose(new Pose());

        TARGET_TAG = 20;

        tele = PanelsTelemetry.INSTANCE.getTelemetry();

        controlA.Add(ControlHandler.Buttons.A, this::switchCentric)
                .onPressed();
        controlA.Add(ControlHandler.Buttons.START, this::requestStart)
                .onPressed();

        controlA.Add(ControlHandler.Buttons.Y, this::switchAlliance)
                .onPressed();
        controlA.Add(ControlHandler.Buttons.B, this::toggleAlign)
                .onPressed();

        cam.start();

        intake = new Intake(hardwareMap, follower);
        shooter = new Shooter(hardwareMap, follower, intake, false);

        clock = new ElapsedTime();

        block = BlockStates.ZERO;

        while (opModeInInit()) {
            if (gamepad1.leftBumperWasPressed()) {
                if (TARGET_TAG == 20) {
                    TARGET_TAG = 24;
                } else {
                    TARGET_TAG = 20;
                }
            }

            telemetry.addLine("Current Alliance: " + (TARGET_TAG == 24 ? "Red" : "Blue"));
            telemetry.addLine("Press LEFT BUMPER to Change");
            telemetry.update();
        }

        Pose start = fromCamera();

        if (start != null) {
            follower.setStartingPose(start);
            follower.setPose(start);
        }

        waitForStart();

        follower.startTeleopDrive(true);
        follower.update();

        flag = 0;

        while (opModeIsActive()) {
            forward = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x;

            double toDebug = 0;
            if (aligning) {
                if (!isCameraOn) {
                    cam.start();
                    isCameraOn = true;
                }

                 Lock.setError(cam.getX(TARGET_TAG, -toDegrees(follower.getHeading())));
                 turn = -Lock.calculate();
                 toDebug = turn;

                 if (Math.abs(toDebug) <= 0.05) {
                     Pose pos = fromCamera();

                     if (pos != null) {
                         follower.setPose(pos);
                     }
                 }
            }
            if (isCameraOn && !aligning) {
                isCameraOn = false;
                cam.stop();
            }

            follower.setTeleOpDrive(forward, strafe, turn, isRobotCentric);


            intake.usingGamepad(gamepad2);
            shooter.usingGamepad(gamepad2);

            intake.update();
            shooter.update();


            velocities.add(shooter.getVelocity());
            if (velocities.size() > 5) {
                velocities.remove(0);
            }

            double stdDev = LazyMath.variance(velocities);

            double elapsed = clock.milliseconds() - last;

            switch (block) {

                case ZERO:
                    blockMotor.setPower(0);

                    if (shooter.getVelocity() > 300
                            && stdDev < 15
                            && intake.getVelocity() > 150) {

                        block = BlockStates.BLOCKING;
                        last = clock.milliseconds();
                    }
                    break;

                case BLOCKING:
                    blockMotor.setPower(-1);

                    if (elapsed >= 275) {
                        block = BlockStates.DEBOUCING;
                        last = clock.milliseconds();
                    }
                    break;

                case DEBOUCING:
                    blockMotor.setPower(0.02);

                    if (elapsed >= 450) {
                        block = BlockStates.ZERO;
                        last = clock.milliseconds();
                    }
                    break;
            }

            controlA.update();
            controlB.update();

            telemetry.addData("Current Debounce: ", String.valueOf(elapsed / 1000) + "s");
            telemetry.addData("Current Shooter Speed: ", shooter.getVelocity());
            telemetry.addData("Current Intake Speed: ", intake.getCurrent());
            telemetry.addData("Current Limelight P: ", toDebug);
            telemetry.addData("Current Limelight D: ", Lock.derivative);
            telemetry.addData("Current Limelight I: ", Lock.integral);
            telemetry.addLine("Current Pose: " + follower.getPose().toString());

            follower.update();
            telemetry.update();
        }
    }

    private void requestStart() {
        if (opModeIsActive()) return;
        start();
    }

    private void switchCentric() {
        isRobotCentric = !isRobotCentric;
    }

    public void switchAlliance() { isRedAlliance = !isRedAlliance; }

    public void toggleAlign() { aligning = !aligning; }

    @Nullable
    public Pose fromCamera() {
        Pose3D current = cam.getPose(TARGET_TAG, follower.getHeading());

        if (current == null) {
            return null;
        }

        return new Pose(current.getPosition().x, current.getPosition().y, current.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public void resetCamera() {
        cam.stop();
    }
}
