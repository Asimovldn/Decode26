package org.firstinspires.ftc.teamcode.TeleOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Systems.Storage;

public class StorageTest extends LinearOpMode {
    public Storage storage;

    public void runOpMode() {
        storage = new Storage();
        storage.setup(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            storage.forcePower(gamepad1.left_stick_x);
        }
    }
}
