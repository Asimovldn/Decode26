package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Math.PIDF;

import java.util.Timer;

public class Shooter {
    public DcMotorEx motor;
    public Servo servo;
    private Follower drive;
    private Mecanum mecanum;
    private PIDF pidShooter;
    private State state;
    private double target, current, lastPIDF;
    private double multiplier;
    private ElapsedTime timer;
    private Intake intake;
    private boolean useServo;


    public Shooter(HardwareMap hm, Follower follower, Intake intake, boolean useServo) {
        motor = hm.get(DcMotorEx.class, "shooter");
        if (useServo) {
            servo = hm.get(Servo.class, "doorServo");
        }

        this.useServo = useServo;

        drive = follower;
        mecanum = (Mecanum) drive.drivetrain;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        this.intake = intake;

        multiplier = 8;

        pidShooter = new PIDF(new PIDFCoefficients(1.5, 0.01, 1.1, 0.3));
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
        if (!useServo) return;
        if (servo.getPosition() == 1) {
            servo.setPosition(0);
        } else {
            servo.setPosition(1);
        }
    }

    public String getServoPosition() {
        if (!useServo) return "No Servo";
        return servo.getPosition() == 1 ? "Open" : "Closed";
    }

    public boolean isBusy() {
        return getVelocity() > 100;
    }

    public Shooter update() {
        current = motor.getVelocity(AngleUnit.DEGREES);


        lastPIDF = pidShooter.calculate(target, current);

        motor.setVelocity(target + lastPIDF, AngleUnit.DEGREES);

        return this;
    }

    public double calculatePIDF() {
        return lastPIDF;
    }

    public double getVelocity() {
        return Math.abs(current);
    }

    public void switchMultiplier() {
        if (multiplier < 6) {
            multiplier = 6;
        } else {
            multiplier = 3;
        }
    }

    public void usingGamepad(Gamepad gamepad2) {
        double m = LazyMath.lerp(410, 515, gamepad2.left_trigger); // 540 max, overkill!
        // 410 min.

        target = m * gamepad2.left_stick_y * (1 - Math.pow(gamepad2.right_trigger, 2));
    }

    public void activateSlowMode() {
        multiplier = 1.5;
        pidShooter.editCoefficients(new PIDFCoefficients(0.5, 0.001, 0.3, 0.03));
    }

    public void usingTrigger(Gamepad gamepad2) {
        current = motor.getVelocity(AngleUnit.DEGREES);

        target = 360 * -gamepad2.right_trigger * multiplier;

        if (target > 360) {
            state = State.SHOOTING;
        } else if (target < -360) {
            state = State.REVERSE;
        } else {
            state = State.HOLD;
        }

        motor.setVelocity(target + pidShooter.calculate(target, current), AngleUnit.DEGREES);
    }

    public void quickShoot() {
        timer.reset();
        while (timer.time() < 3) {
            drive.setTeleOpDrive(0, 0, 0);
        }

        target = 360 * 3;
        timer.reset();
        intake.setState(Intake.State.INTAKE);

        while (timer.time() < 3) update();

        intake.setState(Intake.State.HOLD);
        timer.reset();
        target = 0;

        while (timer.time() < 2) update();

        return;
    }
}
