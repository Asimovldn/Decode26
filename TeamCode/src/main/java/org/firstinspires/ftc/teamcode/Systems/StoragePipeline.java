package org.firstinspires.ftc.teamcode.Systems;

import org.firstinspires.ftc.teamcode.References.ARTIFACT;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class StoragePipeline extends OpenCvPipeline {
    public static Scalar greenLowerBound = new Scalar(30 , 50, 50);
    public static Scalar greenUpperBound = new Scalar(90, 255, 255);

    public static Scalar purpleLowerBound = new Scalar(120, 50, 50);
    public static Scalar purpleUpperBound = new Scalar(160, 255, 255);

    public static Scalar yellowLowerBound = new Scalar(20, 50, 50);
    public static Scalar yellowUpperBound = new Scalar(50, 255, 255);

    public static String maskToShow = "input";



    private ARTIFACT areaState = ARTIFACT.NONE;

    Mat hsv = new Mat();
    Mat greenMask = new Mat();
    Mat purpleMask = new Mat();
    Mat yellowMask = new Mat();

    Mat greenCrop;
    Mat purpleCrop;
    Mat yellowCrop;

    Mat output = new Mat();

    int greenCount = 0;
    int purpleCount = 0;
    int yellowCount = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, greenLowerBound, greenUpperBound, greenMask);
        Core.inRange(hsv, purpleLowerBound, purpleUpperBound, purpleMask);
        Core.inRange(hsv, yellowLowerBound, yellowUpperBound, yellowMask);

        Rect rect = new Rect((320 / 2) - 70, 240 - 100, 170, 100);

        greenCrop = greenMask.submat(rect);
        purpleCrop = purpleMask.submat(rect);
        yellowCrop = yellowMask.submat(rect);

        greenCount = Core.countNonZero(greenCrop);
        purpleCount = Core.countNonZero(purpleCrop);
        yellowCount = Core.countNonZero(yellowCrop);

        Imgproc.rectangle(input, rect, new Scalar(255,0,0), 2);

        switch (maskToShow)
        {
            case "input":
                input.copyTo(output);
                break;

            case "green":
                greenCrop.copyTo(output);
                break;

            case "purple":
                purpleCrop.copyTo(output);
                break;
        }


        hsv.release();
        greenMask.release();
        purpleMask.release();
        yellowMask.release();
        greenCrop.release();
        purpleCrop.release();
        yellowCrop.release();


        return output;
    }

    public ARTIFACT getAreaState()
    {

        double max = Math.max(Math.max(yellowCount, purpleCount), greenCount);

        if (max == purpleCount)
        {
            return ARTIFACT.PURPLE;
        } else if (max == greenCount) {
            return ARTIFACT.GREEN;
        } else {
            return ARTIFACT.NONE;
        }
    }

    public boolean readingBall() {
        return getAreaState() == ARTIFACT.PURPLE || getAreaState() == ARTIFACT.GREEN;
    }
}
