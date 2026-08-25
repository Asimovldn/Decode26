package org.firstinspires.ftc.teamcode.TeleOpModes;

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

import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Math.PID;
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
    Limelight3A cam;
    boolean useCam = false;
    boolean aligning;

    private boolean isRobotCentric = true, isRedAlliance = true;

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

        cam = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (cam == null) {
            useCam = false;
        }

        velocities = new ArrayList<>();

        follower.setStartingPose(new Pose());

        tele = PanelsTelemetry.INSTANCE.getTelemetry();

        controlA.Add(ControlHandler.Buttons.A, this::switchCentric)
                .onPressed();
        controlA.Add(ControlHandler.Buttons.START, this::requestStart)
                .onPressed();

        controlA.Add(ControlHandler.Buttons.Y, this::switchAlliance)
                .onPressed();
        controlA.Add(ControlHandler.Buttons.RIGHTSTICK, this::toggleAlign)
                .onPressed();

        intake = new Intake(hardwareMap, follower);
        shooter = new Shooter(hardwareMap, follower, intake, false);

        clock = new ElapsedTime();

        block = BlockStates.ZERO;

        waitForStart();

        follower.startTeleopDrive(true);
        follower.update();

        flag = 0;

        while (opModeIsActive()) {
            forward = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x;

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

                    if (elapsed >= 100) {
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

            telemetry.addData("Current Block State: ", block.toString());
            telemetry.addData("Current Debounce: %.2f s", elapsed / 1000);
            telemetry.addData("Current Shooter StdDev: ", stdDev);
            telemetry.addData("Current Shooter Speed: ", shooter.getVelocity());
            telemetry.addData("Current Intake Speed: ", intake.getCurrent());

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

    private void initCamera() {
        if (!useCam) return;

        cam.pipelineSwitch(0);
        cam.start();
    }

    @Nullable
    private Double fromCamera() {
        if (!useCam) return null;

        LLResult res = cam.getLatestResult();
        if (res == null || !res.isValid()) {
            return null;
        }

        LLResultTypes.FiducialResult target = null;

        for (LLResultTypes.FiducialResult fiducial :
                res.getFiducialResults()) {
            if (fiducial.getFiducialId() == TARGET_TAG) {
                target = fiducial;
                break;
            }
        }

        assert target != null : "Target fromCamera() não identificado/nulo.";
        return target.getTargetXDegrees();
    }

    public void switchAlliance() { isRedAlliance = !isRedAlliance; }

    public void toggleAlign() { aligning = !aligning; }
}
