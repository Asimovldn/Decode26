package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    public Intake(HardwareMap hm, Follower follower) {
        motor = hm.get(DcMotorEx.class, "intake");
        drive = follower;
        mecanum = (Mecanum) drive.drivetrain;

        pidIntake = new PIDF(new PIDFCoefficients(1.1, 0.01, 0.7, 0.1), 0.005);
    }

    public Intake(HardwareMap hm, Shooter shooter, Follower follower) {
        this(hm, follower);
        shootingSystem = Outtake.SHOOTER;
    }

    public Intake(HardwareMap hm, Turret turret, Follower follower) {
        this(hm, follower);
        shootingSystem = Outtake.TURRET;
    }

    public State getState() {
        return state;
    }

    public enum Outtake {
        SHOOTER,
        TURRET
    }

    public enum State {
        INTAKE,
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

    public void update() {
        current = motor.getVelocity(AngleUnit.DEGREES);

        if (state == State.INTAKE) {
            target = 360 * 4;
        } else if (state == State.OUTTAKE) {
            target = -360 * 4;
        } else {
            target = 0;
        }

        motor.setVelocity(pidIntake.calculate(getCurrent(), getTarget()), AngleUnit.DEGREES);
    }

    public void usingGamepad(Gamepad gamepad2) {
        current = motor.getVelocity(AngleUnit.DEGREES);

        target = 360 * -gamepad2.right_stick_y * 3;

        motor.setVelocity(pidIntake.calculate(current, target), AngleUnit.DEGREES);
    }
}
