package org.firstinspires.ftc.teamcode.Autonomie;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline_Red extends OpenCvPipeline
{
    public enum SkystonePosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    static volatile SkystonePosition position = SkystonePosition.CENTER;
    Mat maskL = new Mat();
    Mat maskC = new Mat();
    Mat maskR = new Mat();
    Mat hsvframe = new Mat();

    @Override
    public void init(Mat firstFrame)
    {
        processFrame(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, hsvframe, Imgproc.COLOR_RGB2HSV);

        Rect LEFT_REGION = new Rect(1, 1, 425, 719);
        Rect CENTER_REGION = new Rect(426, 1, 425, 719);
        Rect RIGHT_REGION = new Rect(852, 1, 425, 719);

        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);

        Mat regionL = hsvframe.submat(LEFT_REGION);
        Mat regionC = hsvframe.submat(CENTER_REGION);
        Mat regionR = hsvframe.submat(RIGHT_REGION);

        Core.inRange(regionL, lowerRed, upperRed, maskL);
        Core.inRange(regionC, lowerRed, upperRed, maskC);
        Core.inRange(regionR, lowerRed, upperRed, maskR);

        double regionLRed = Core.countNonZero(maskL) / (regionL.width() * regionL.height() / 100.0);
        double regionCRed = Core.countNonZero(maskC) / (regionC.width() * regionC.height() / 100.0);
        double regionRRed = Core.countNonZero(maskR) / (regionR.width() * regionR.height() / 100.0);

        double maxi = Math.max(regionLRed, Math.max(regionCRed, regionRRed));
        if (maxi == regionLRed)
            position = SkystonePosition.LEFT;
        else
            if (maxi == regionCRed)
                position = SkystonePosition.CENTER;
            else
                position = SkystonePosition.RIGHT;

        return input;
    }

    public SkystonePosition getAnalysis()
    {
        return position;
    }
}