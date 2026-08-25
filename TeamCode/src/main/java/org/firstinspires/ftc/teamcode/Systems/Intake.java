package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Math.PIDF;

public class Intake {
    public Outtake shootingSystem;
    public DcMotorEx motor;
    private Follower drive;
    private Mecanum mecanum;
    private PIDF pidIntake;
    private State state;
    private double target, current;
    private boolean wasIntaked, detect;

    private Storage storage;

    public Intake(HardwareMap hm, Follower follower) {
        motor = hm.get(DcMotorEx.class, "intake");
        drive = follower;
        mecanum = (Mecanum) drive.drivetrain;

        pidIntake = new PIDF(new PIDFCoefficients(0.3, 0.005, 0.1, 0.01));
    }

    public Intake(HardwareMap hm, Shooter shooter, Follower follower) {
        this(hm, follower);
        shootingSystem = Outtake.SHOOTER;
    }

    public Intake(HardwareMap hm, Turret turret, Follower follower) {
        this(hm, follower);
        shootingSystem = Outtake.TURRET;
    }

    public Intake(HardwareMap hm, Storage storage, Follower follower) {
        this(hm, follower);
        this.storage = storage;
    }

    public State getState() {
        return state;
    }

    public boolean intaked() {
       detect = storage.readingBall(); // reach for camera readings here.
       boolean intaked = detect && !wasIntaked;
       wasIntaked = detect;

       return intaked;
    }

    public enum Outtake {
        SHOOTER,
        TURRET
    }

    public enum State {
        INTAKE,
        SLOWINTAKE,
        OUTTAKE,
        HOLD
    }

    public void setState(State newState) {
        state = newState;
    }

    public void forceBrake() {
        motor.setVelocity(0);
    }

    public double getTarget() { return target; }
    public double getCurrent() { return current; }

    public double getVelocity() {
        return Math.abs(motor.getVelocity());
    }

    public double getCorrente() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public Intake update() {
        current = motor.getVelocity(AngleUnit.DEGREES);

        motor.setVelocity(target + pidIntake.calculate(target, current), AngleUnit.DEGREES);
        return this;
    }

    public void usingGamepad(Gamepad gamepad2) {
        current = motor.getVelocity(AngleUnit.DEGREES);

        target = 360 * -gamepad2.right_stick_y * 1;
    }

    public void usingTrigger(Gamepad gamepad2) {
        current = motor.getVelocity(AngleUnit.DEGREES);

        target = 360 * Math.pow(gamepad2.left_trigger, 2);

        motor.setVelocity(target + pidIntake.calculate(current, target), AngleUnit.DEGREES);
    }
}
