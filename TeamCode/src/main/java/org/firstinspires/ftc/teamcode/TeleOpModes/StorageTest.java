package org.firstinspires.ftc.teamcode.TeleOpModes;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class StorageTest extends LinearOpMode {
    public Storage storage;
    public Intake intake;
    public Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap, follower);
        storage = new Storage();

        storage.setup(hardwareMap, gamepad1);
        storage.openCamera();

        waitForStart();

        while (opModeIsActive()) {
            storage.update();
            intake.usingGamepad(gamepad1);
        }
    }
}
