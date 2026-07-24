package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.List;

public class LimelightHandler
{
    private static final String LIMELIGHT_URL = "http://limelight.local:5807/target";

    private Limelight3A limelight;

    enum PipelineList
    {
        APRILTAG(7);

        public final int id;

        private PipelineList(int id)
        {
            this.id = id;
        }
    };

    public void init(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PipelineList.APRILTAG.id);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public HashMap<String, Double> getAprilTagData(int id)
    {
        LLResult result = limelight.getLatestResult();
        if (!(result != null && result.isValid()))
        {
            return null;
        }

        if (result.getFiducialResults().isEmpty())
        {
            return null;
        }
        HashMap<String, Double> map = new HashMap<>();

        List<FiducialResult> fiducials = result.getFiducialResults();

        FiducialResult fiducial = null;
        for (FiducialResult Ifiducial : fiducials)
        {
            if (Ifiducial.getFiducialId() == id)
            {
                fiducial = Ifiducial;
                break;
            }
        }

        if (fiducial == null)
        {
            return null;
        }
        double tx = fiducial.getTargetXDegrees(); // Where it is (left-right)
        double ty = fiducial.getTargetYDegrees(); // Where it is (up-down)
        double ta = fiducial.getTargetArea();

        map.put("tx", tx);
        map.put("ty", ty);
        map.put("ta", ta);
        map.put("id", (double)id);

        return map;
    }

}
