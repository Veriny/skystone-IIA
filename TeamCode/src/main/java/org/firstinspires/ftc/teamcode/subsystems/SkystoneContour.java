package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Telemetry tele;
    private double x, y;
    private List<MatOfPoint> contours = new ArrayList<>();
    private boolean showContours, isAccessible = true;
    private double mThing;
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
            mThing = m.m10/m.m00;
        }
        return xpos;
    }

    public double getmThing() {
        return mThing;
    }

    public void setTelemetry(Telemetry tele) {
        this.tele = tele;
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

    public synchronized int getContourFoundCount() {
        return contours.size();
    }

    public boolean skystoneIsCentered() {
        boolean xCoordCentered = false;
        boolean yCoordCentered = false;
         while (!isAccessible) {
             //do nothing
         }
        ArrayList<Double> xPositions = getContourXPos();
        ArrayList<Double> yPositions = getContourYPos();
        if(xPositions.size() == 0 || yPositions.size() == 0) {
            return false;
        }
        for(int contourX = 0; contourX < xPositions.size(); contourX++) {
            if(xPositions.get(contourX) > 140 && 180 > xPositions.get(contourX)) {
                xCoordCentered = true;
            }
        }
        for(int contourY = 0; contourY < yPositions.size(); contourY++) {
            if(yPositions.get(contourY) > 100 && 140 > yPositions.get(contourY)) {
                yCoordCentered = true;
            }
        }
        return xCoordCentered && yCoordCentered;
    }

    @Override
    public Mat processFrame(Mat input) {
        isAccessible = false;
        Imgproc.cvtColor(input, yuv, Imgproc.COLOR_RGB2HSV, 3);
        //Create a binary image with the upper and lower bounds of the colors we want.
        Core.inRange(yuv, new Scalar(0, 0, 0), new Scalar(180,255,34), bimImg);
        //Now, we erode the binary image to get rid of any dirtiness.
        structElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        //Eroding the image allows us to better locate the skystone, as the border walls of the playing field are black as well
        Imgproc.erode(bimImg, bimImg, structElement);
        Imgproc.findContours(bimImg, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (showContours && contours.size() > 0) {
            Imgproc.drawContours(bimImg, contours, -1, new Scalar(255, 255, 255), 2, 8);
        }
        isAccessible = true;
        return yuv;
    }

}
