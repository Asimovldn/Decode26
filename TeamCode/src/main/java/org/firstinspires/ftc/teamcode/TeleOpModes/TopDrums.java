package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.PIDF;

@TeleOp
public class TopDrums extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx flavioWheel = hardwareMap.get(DcMotorEx.class, "storage");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        PIDF fuckingPID = new PIDF(new PIDFCoefficients(1.2, 0.01, 0.7, 0.4));

        waitForStart();

        while (opModeIsActive()) {
            flavioWheel.setPower(-gamepad1.right_trigger);

            double target = gamepad1.right_stick_y * 1680;

            shooter.setVelocity(target + fuckingPID.angles(target, shooter.getVelocity(AngleUnit.DEGREES)));

            intake.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Rodinha Verde", flavioWheel.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Shooter", shooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Intake", intake.getVelocity(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
