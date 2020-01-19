package org.firstinspires.ftc.teamcode.Subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class SkystoneContour extends OpenCvPipeline {
    private Mat yuv = new Mat();
    private Mat bimImg = new Mat();
    private Mat structElement = new Mat();
    private double x, y;
    private List<MatOfPoint> contours = new ArrayList<>();
    private boolean showContours = false;
    public synchronized void setShowContours(boolean b) {
        showContours = b;
    }
    public synchronized ArrayList<Double> getContourXPos() {
//        Moments m = Imgproc.moments(contours.get(contours.size() - 1));
//        //here, we return the average x position of the contour
//        return m.m10/m.m00;
        Moments m;
        ArrayList<Double> xpos = new ArrayList<>();
        for (MatOfPoint mat : contours) {
            m = Imgproc.moments(mat);
            xpos.add(m.m10/m.m00);
        }
        return xpos;
    }

    public synchronized ArrayList<Double> getContourYPos() {
//        Moments m = Imgproc.moments(contours.get(contours.size() - 1));
//        //here, we return the average y position of the contour
//        return m.m01/m.m00;
        Moments m;
        ArrayList<Double> ypos = new ArrayList<>();
        for (MatOfPoint mat : contours) {
            m = Imgproc.moments(mat);
            ypos.add(m.m01/m.m00);
        }
        return ypos;
    }



    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, yuv, Imgproc.COLOR_RGB2YUV, 3);
        //Create a binary image with the upper and lower bounds of the colors we want.
        Core.inRange(yuv, new Scalar(0, 0, 0), new Scalar(20, 20, 20), bimImg);
        //Now, we erode the binary image to get rid of any dirtiness.
        structElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        //Eroding the image allows us to better locate the skystone, as the border walls of the playing field are black as well
        Imgproc.erode(bimImg, bimImg, structElement);
        Imgproc.findContours(bimImg, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (showContours) {
            Imgproc.drawContours(bimImg, contours, -1, new Scalar(0, 255, 0), 2, 8);
        }
        return yuv;
    }
}
