package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Configurable
public class Kronos3Auto{
    private Limelight3A cam;
    private Follower follower;
}
