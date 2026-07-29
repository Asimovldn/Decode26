package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.Framework;
import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.References.ARTIFACT;
import org.firstinspires.ftc.teamcode.References.StorageUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.HashMap;

public class Storage {

    DcMotorEx motor;
    Framework control;
    StorageUtils.Position current, target;
    HashMap<String, Double> pos = new HashMap<>();

    private final int TICK_PER_REV = 560;
    private final double TICK_PER_DEGREE = Math.round(TICK_PER_REV / 360.0);

    private double debounce = 200;
    private Shooter shooter;

    private double lastTick, currentTick;
    private ElapsedTime time;
    private boolean hasTimeStarted;

    ARTIFACT currentShooterArtifact = ARTIFACT.NONE;

    public void setup(HardwareMap hardwareMap, Shooter shooter) {
        motor = hardwareMap.get(DcMotorEx.class, "storage");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        control = new Framework(new PIDFCoefficients(1, 0.1, 0.5, 0.2), new PIDCoefficients(0.4, 0.01, 0.3), 1.2, 3);

        current = StorageUtils.Position.INTAKE;
        target = StorageUtils.Position.BLANK;
        pos.put("INTAKE", 0.0);
        pos.put("SHOOTER", -60.0);
        pos.put("BACKUP", 120.0);

        currentTick = motor.getCurrentPosition();
        lastTick = currentTick;

        time = new ElapsedTime();
        hasTimeStarted = false;

        control.setMemorySize(15);
        control.assignAverageError(60);
    }

    public void startTimer() {
        hasTimeStarted = true;
        time.reset();
    }

    public void forcePower(double pwr) {
        motor.setPower(pwr);
    }

    public void goTo(StorageUtils.Position position) {
        target = position;
    }

    public void update() {
        if (target != current) {
            currentTick = motor.getCurrentPosition();

            double currentAngle = motor.getCurrentPosition() * 360.0 / TICK_PER_REV;
            double dTicks = lastTick - currentTick;

            control.useAsDerivative((dTicks * 360) / (TICK_PER_REV * time.seconds()));

            double output = control.calculate(pos.get(target.toString()), currentAngle);

            motor.setVelocity(output);
            time.reset();

            lastTick = currentTick;
        }
    }

    public void shoot() {
        if (shooter.getVelocity() > 1000) {

        }
    }
}
