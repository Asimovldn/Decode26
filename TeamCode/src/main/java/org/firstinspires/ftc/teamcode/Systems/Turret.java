package org.firstinspires.ftc.teamcode.Systems;

import android.os.Process;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Math.PIDF;
import org.openftc.apriltag.AprilTagDetection;

import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
Controls Turret position, tasks and shooting.
 Utilizes a PD Control (Proportional Derivative) for turret position.
 */
@Configurable
public class Turret {
    @IgnoreConfigurable
    private DcMotorEx motor;

    @IgnoreConfigurable
    private DcMotorEx angulator;

    @IgnoreConfigurable
    private double lastErr = 0;

    @IgnoreConfigurable
    private PIDF pidTurret;

    @IgnoreConfigurable
    private double angLastErr = 0;

    @IgnoreConfigurable
    private double power = 0;

    @IgnoreConfigurable
    private final ElapsedTime time = new ElapsedTime();

    @IgnoreConfigurable
    private double currentAngulatorAngle = 45;

    @Configurable
    private static class TurretConstants {
        public static double kP = 0;
        public static double kD = 0;

        public static double angleTolerance = 0.15;

        public static double goalX = 0;
        public static final double MAX_POWER = 0.6;

        //  Angulator
        public static final double CPR = 560;
        public static final double motorTeeth = 20;
        public static final double angulatorTeeth = 100;

        public static double ANGULATOR_MAX_POWER = 0.6;
        public static double angulatorTolerance = 1.5;

        // Clicks Per Degree
        @IgnoreConfigurable
        public static final double CPD = (CPR * angulatorTeeth) / (motorTeeth * 360);
    }

    private void setup(HardwareMap hardwareMap, TelemetryManager telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angulator = hardwareMap.get(DcMotorEx.class, "AngulatorMotor");
        angulator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        adjustPDValues(0.1, 0.08);

        pidTurret = new PIDF(new PIDFCoefficients(kp.get(), 0, kp.get(), 0.01));
    }

    /**
     * Adjusts PD Values, -1 means that it'll remain the same.
     * @param kP the new KP Value.
     * @param kD the new KD Value.
     */
    public void adjustPDValues(double kP, double kD) {
        if (kP != -1) TurretConstants.kP = kP;
        if (kD != -1) TurretConstants.kD = kD;

        pidTurret.editCoefficients(new PIDFCoefficients(kp.get(), 0, kd.get(), 0.01));
    }

    public Supplier<Double> kp = () -> TurretConstants.kP;
    public Supplier<Double> kd = () -> TurretConstants.kD;

    /**
     * Calculates the angle needed for the ball to travel into the target considering
     * @return the angle needed.
     */
    public double calculateAngulator(double dt, AprilTagDetection currentID, Pose robotPose) {
        currentAngulatorAngle = motor.getCurrentPosition() * TurretConstants.CPD;

        double target = Math.toDegrees(
                Math.atan2(currentID.pose.z, currentID.pose.z)
        );

        double err = target - currentAngulatorAngle;


        if (Math.abs(err) < TurretConstants.angulatorTolerance) {
            power = 0;
        } else {
            power = Range.clip(pidTurret.calculate(currentAngulatorAngle, target), -TurretConstants.ANGULATOR_MAX_POWER, TurretConstants.ANGULATOR_MAX_POWER);
        }

        return power;
    };

    public void update(AprilTagDetection currentID, Pose pose) {
        double dt = time.seconds();
        time.reset();

        if (currentID == null) {
            motor.setPower(0);
            lastErr = 0;
            return;
        }

        // Essa versão do FTC SDK não tem ftcPose.bearing. O Bearing deve ser calculado manualmente. (math.todegrees(math.atan2(...));
        double err = TurretConstants.goalX - Math.toDegrees(Math.atan2(currentID.pose.x, currentID.pose.z));

        double p = err * kp.get();
        double d = 0;

        if (dt > 0) d = ((err - lastErr) / dt) * kd.get();

        double simpleErr = LazyMath.simplify(err, -360, 360);

        if (Math.abs(err) < TurretConstants.angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(p + d, -TurretConstants.MAX_POWER, TurretConstants.MAX_POWER);
        }

        motor.setVelocity(power * 360);
        angulator.setPower(calculateAngulator(dt, currentID, pose));
    }

}