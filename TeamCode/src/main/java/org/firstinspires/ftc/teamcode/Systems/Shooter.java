package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Math.PIDF;

public class Shooter {
    public DcMotorEx motor;
    public Servo servo;
    private Follower drive;
    private Mecanum mecanum;
    private PIDF pidShooter;
    private State state;
    private double target, current;
    private double multiplier;


    public Shooter(HardwareMap hm, Follower follower) {
        motor = hm.get(DcMotorEx.class, "shooter");
        servo = hm.get(Servo.class, "doorServo");
        drive = follower;
        mecanum = (Mecanum) drive.drivetrain;

        multiplier = 5;

        pidShooter = new PIDF(new PIDFCoefficients(1.1, 0.01, 0.7, 0.1), 0.005);
    }


    public enum State {
        SHOOTING,
        REVERSE,
        HOLD
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void setMultiplier(double nMult) {
        multiplier = nMult;
    }

    public double getMultiplier() { return multiplier; }

    public void forceBrake() {
        motor.setVelocity(0);
    }

    public double getTarget() { return target; }
    public double getCurrent() { return current; }

    public void switchServo() {
        if (servo.getPosition() == 1) {
            servo.setPosition(0);
        } else {
            servo.setPosition(1);
        }
    }

    public String getServoPosition() {
        return servo.getPosition() == 1 ? "Open" : "Closed";
    }

    public void update() {
        current = motor.getVelocity(AngleUnit.DEGREES);

        if (state == State.SHOOTING) {
            target = 360 * 7;
        } else if (state == State.REVERSE) {
            target = -360 * 7;
        } else {
            target = 0;
        }

        motor.setVelocity(pidShooter.calculate(getCurrent(), getTarget()), AngleUnit.DEGREES);
    }

    public void switchMultiplier() {
        if (multiplier == 5) {
            multiplier = 8;
        } else {
            multiplier = 5;
        }
    }

    public void usingGamepad(Gamepad gamepad2) {
        current = motor.getVelocity(AngleUnit.DEGREES);

        target = 360 * -gamepad2.left_stick_y * multiplier;

        if (target > 360) {
            state = State.SHOOTING;
        } else if (target < -360) {
            state = State.REVERSE;
        } else {
            state = State.HOLD;
        }

        motor.setVelocity(pidShooter.calculate(current, target), AngleUnit.DEGREES);
    }

    public void usingTrigger(Gamepad gamepad2) {
        current = motor.getVelocity(AngleUnit.DEGREES);

        target = 360 * gamepad2.right_trigger * multiplier;

        if (target > 360) {
            state = State.SHOOTING;
        } else if (target < -360) {
            state = State.REVERSE;
        } else {
            state = State.HOLD;
        }

        motor.setVelocity(pidShooter.calculate(current, target), AngleUnit.DEGREES);
    }
}
