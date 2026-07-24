package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.LimelightHandler;

import java.util.HashMap;

@TeleOp
public class AprilTagTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        LimelightHandler limelightHandler = new LimelightHandler();

        limelightHandler.init(hardwareMap);


        waitForStart();

        while (opModeIsActive())
        {
            HashMap<String, Double> map = limelightHandler.getAprilTagData(22);

            if (map == null)
                continue;

            telemetry.addData("tx: ", map.get("tx"));
            telemetry.addData("ty: ", map.get("ty"));
            telemetry.addData("ta", map.get("ta"));
            telemetry.addData("id", map.get("id"));
            telemetry.update();
        }
    }
}
