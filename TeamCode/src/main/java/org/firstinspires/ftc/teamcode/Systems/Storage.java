package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.math.MathFunctions.getSmallestAngleDifference;

import static java.lang.Math.toRadians;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.Framework;
import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Math.PIDF;
import org.firstinspires.ftc.teamcode.References.ARTIFACT;
import org.firstinspires.ftc.teamcode.References.StorageUtils;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.HashMap;

public class Storage {

    DcMotorEx motor;
    PIDF control;
    StorageUtils.Position target;
    HashMap<String, Double> pos = new HashMap<>();
    States state;
    Gamepad gamepad;

    private final int TICK_PER_REV = 560;
    private final double TICK_PER_DEGREE = TICK_PER_REV / 360.0;

    private double debounce, lastTime = 200;
    private Shooter shooter;

    private double lastTick, currentTick, current;
    private ElapsedTime time;
    private boolean hasTimeStarted, wasShooting, isBusy, flag;

    ARTIFACT currentShooterArtifact = ARTIFACT.NONE;

    OpenCvCamera camera;
    StoragePipeline pipeline;


    public enum States {
      IDLE,
      SHOOTING,
      RETURNING,
      BRAKING;
    };

    public void setup(HardwareMap hm, Gamepad gp) {
        setup(hm);
        gamepad = gp;
    }

    public void setup(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "storage");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        control = new PIDF(new PIDFCoefficients(1.2, 0.01, 0.8, 0.2));

        target = StorageUtils.Position.BLANK;
        pos.put("INTAKE", 0.0);
        pos.put("SHOOTER", -240.0);
        pos.put("BACKUP", -120.0);
        pos.put("BLANK", 0.0);
        pos.put("BRAKE", 0.0);

        current = pos.get("INTAKE");

        currentTick = motor.getCurrentPosition();
        lastTick = currentTick;

        time = new ElapsedTime();
        hasTimeStarted = false;

        control.setMemorySize(15);

        flag = false;
    }

    public double read(StorageUtils.Position position) {
        return pos.get(position.toString());
    }

    public double radians(StorageUtils.Position position) {
        return toRadians(read(position));
    }

    public void setup(HardwareMap hardwareMap, Shooter shooter) {
        setup(hardwareMap);
        this.shooter = shooter;
    }

    public void setup(HardwareMap hardwareMap, Shooter shooter, String wbcam) {
        setup(hardwareMap, shooter);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, wbcam);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
    }

    public boolean readingBall() {
        return pipeline.readingBall();
    }

    public void openCamera() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                pipeline = new StoragePipeline();
                camera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {};
        });
    }

    public void startTimer() {
        hasTimeStarted = true;
        time.reset();
    }

    public void forcePower(double pwr) {
        motor.setPower(pwr);
    }

    public void goTo(StorageUtils.Position position) {
        if (isBusy) return;
        target = position;
        state = States.RETURNING;
    }

    public void update() {
        if (shooter == null) {
            if (gamepad.aWasPressed()) {
                target = next();
                if (Math.abs(read(target) - current) > 10) {
                    current = motor.getCurrentPosition() / TICK_PER_DEGREE;
                    double dTicks = lastTick - currentTick;
                    double output = control.angles(radians(target), toRadians(current));

                    output = Math.toDegrees(output);
                    motor.setVelocity(output);

                    lastTick = currentTick;
                    lastTime = time.milliseconds();
                } else if (Math.abs(read(target) - current) >= 2) {
                    if (read(target) - current < 0) {
                        motor.setVelocity(-45);
                    } else if (read(target) - current > 0) {
                        motor.setVelocity(45);
                    }
                }

                return;
            }
        }

        double tAngle = read(target);
        current = motor.getCurrentPosition() / TICK_PER_DEGREE;

        if (target == StorageUtils.Position.BRAKE) {
            state = States.BRAKING;
        }

        if (Math.abs(tAngle - current) > 10) {
            currentTick = motor.getCurrentPosition();
            double dTicks = lastTick - currentTick;
            double output = control.angles(radians(target), toRadians(current));

            output = Math.toDegrees(output);
            motor.setVelocity(output);

            lastTick = currentTick;
            lastTime = time.milliseconds();

        } else if (Math.abs(tAngle - current) >= 2) {
            if (tAngle - current < 0) {
                motor.setVelocity(-180);
            } else if (tAngle - current > 0) {
                motor.setVelocity(180);
            }
        }

        current = motor.getCurrentPosition() / TICK_PER_DEGREE;
    }

    public double getDegrees() {
        return current;
    }

    public void execFlag(Consumer<Void> executable) {
        if ((time.milliseconds() - lastTime < debounce)) {
            executable.accept(null);
            flag = true;
        } else { flag = false; }
    }

    public void expectBrake() {
        if (flag) {
            execFlag((Void x) -> {
                state = States.BRAKING;
            });

            isBusy = false;
            goTo(StorageUtils.Position.INTAKE);
        }

        if (shooter.getVelocity() < 300 && wasShooting && !flag) {
            target = StorageUtils.Position.BRAKE;
            wasShooting = false;

            execFlag((Void x) -> {
               state = States.BRAKING;
            });
        }
    }

    public StorageUtils.Position next() {
        if (current - pos.get("SHOOTER") < 3) return StorageUtils.Position.BACKUP;
        if (current - pos.get("BLANK") < 3) return StorageUtils.Position.INTAKE;
        if (current - pos.get("INTAKE") < 3) return StorageUtils.Position.SHOOTER;
        if (current - pos.get("BACKUP") < 3) return StorageUtils.Position.INTAKE;
        return StorageUtils.Position.BLANK;
    }

    public void randandan() {
        if (state == States.IDLE) {
            state = States.SHOOTING;
            shoot();
        }
    }

    public void shoot() {
        if (flag) {
            execFlag((Void x) -> {
                update();
            });
            expectBrake();
        }

        if (shooter.getVelocity() > 1000 && !flag) {
            wasShooting = true;
            isBusy = true;
            target = next();
            execFlag((Void x) -> {
                update();
            });
            expectBrake();
        }
    }
}
