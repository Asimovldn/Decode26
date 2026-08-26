package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Math.LazyMath;
import org.jetbrains.annotations.Nls;
import org.jetbrains.annotations.Nullable;

public class Camera {
    @Nullable
    private Limelight3A cam;

    private double lastTagAngle, lastRobotHeading, lastPredict;

    private boolean hasSeenTag = false;

    public Camera(HardwareMap hm) {
        cam = hm.tryGet(Limelight3A.class, "limelight");
    }

    @Nullable
    public Camera start() {
        if (cam == null) return null;

        cam.start();
        cam.pipelineSwitch(0);

        return this;
    }

    public void stop() {
        if (cam == null) return;
        cam.shutdown();
    }

    public double getX(int APRIL_TAG_ID, double heading) {
        if (cam == null) return -1;

        LLResult res = cam.getLatestResult();

        for (LLResultTypes.FiducialResult result : res.getFiducialResults()) {
            if (result.getFiducialId() != APRIL_TAG_ID) continue;

            lastTagAngle = result.getTargetXDegrees();
            lastPredict = lastTagAngle;

            lastRobotHeading = heading;
            hasSeenTag = true;

            return lastTagAngle;
        }

        if (!hasSeenTag) {
            return 0;
        }

        double headingChange = LazyMath.angleDifference(heading, lastRobotHeading);

        lastRobotHeading = heading;

        lastPredict -= headingChange;

        return lastPredict;
    }

    public double getY(int APRIL_TAG_ID) {
        if (cam == null) return -1;

        LLResult res = cam.getLatestResult();

        for (LLResultTypes.FiducialResult result : res.getFiducialResults()) {
            if (result.getFiducialId() != APRIL_TAG_ID) continue;

            Pose3D pose = result.getTargetPoseCameraSpace();

            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            return Math.sqrt(x * x + y * y + z * z);
        }

        return 0;
    }

    @Nullable
    public Pose3D getPose(int APRIL_TAG_ID, double heading) {
        if (cam == null) return null;

        cam.updateRobotOrientation(Math.toDegrees(heading));


        LLResult res = cam.getLatestResult();

        if (!res.isValid()) return null;

        return res.getBotpose();
    }
}

