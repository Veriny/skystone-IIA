package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
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
    private Mat thresholded = new Mat();
    private Mat structElement = new Mat();
    private MatOfPoint tempMat;
    private Telemetry tele;
    private double x, y, ADJUSTED_X, ADJUSTED_Y;
    private boolean pos1, pos2, pos3;
    private boolean found1, found2, found3;
    private int width, height;
    private double skystoneXPos, skystoneYPos;
    //Important
    private double UPPER_X = 240;
    private double LOWER_X = 80;
//    private double pos1Left_X = 20;
//    private double pos1Right_X = 120;
//    private double pos2Left_X = 120;
//    private double pos2Right_X = 220;
//    private double pos3Left_X = 220;
//    private double pos3Right_X = 320;
    private double UPPER_Y = 65;
    private double LOWER_Y = 35;
    private List<MatOfPoint> contours = new ArrayList<>();
    private boolean showContours, isAccessible = true;
    private boolean found, skystoneIsCentered = false;
    private Moments bruh;
    private int contoursFound;
    private double mThing;
    private double maximumArea = 9000;
    private double minimumArea = 15000;

    //upper and lower bounds for the contours



    public void setShowContours(boolean b) {
        showContours = b;
    }



    @Override
    public Mat processFrame(Mat input) {
        width = input.width();
        height = input.height();
        isAccessible = false;
        showContours = true;
        yuv = input.clone();
        Imgproc.cvtColor(input, yuv, Imgproc.COLOR_BGR2HSV);
        //Create a binary image with the upper and lower bounds of the colors we want.
        Core.inRange(yuv, new Scalar(0, 0, 0), new Scalar(255,255,90), thresholded);
        //Now, we erode the binary image to get rid of any dirtiness.
//        structElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        //Eroding the image allows us to better locate the skystone, as the border walls of the playing field are black as well
        Imgproc.erode(thresholded, thresholded, structElement);
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            if (showContours)
                Imgproc.drawContours(thresholded, contours, -1, new Scalar(0, 0, 0), 2, 8);
            contoursFound = contours.size();
            for (int i = 0; i < contours.size(); i++) {
                tempMat = contours.get(i);
                Rect boundingRect = Imgproc.boundingRect(tempMat);
                bruh =  Imgproc.moments(tempMat);
                x = bruh.m10/bruh.m00;
                y = bruh.m01/bruh.m00;
                ADJUSTED_X = boundingRect.tl().x;
                ADJUSTED_Y = boundingRect.tl().y;
                double matArea = Imgproc.contourArea(tempMat);
                boolean matAreaInBounds = (matArea > 12000) && (matArea < 28000);

                if (//x > LOWER_X && x < UPPER_X &&
                        y > LOWER_Y && y < UPPER_Y && matAreaInBounds||
                        ADJUSTED_X > LOWER_X && ADJUSTED_X < UPPER_X &&
                        ADJUSTED_Y > LOWER_Y && ADJUSTED_Y < UPPER_Y && matAreaInBounds)

                {
                    found = true;
                    skystoneXPos = x;
                    skystoneYPos = y;

//                    if(x > pos1Left_X && x < pos1Right_X ||
//                            ADJUSTED_X > pos1Left_X && ADJUSTED_X < pos1Right_X)
//                    {
//                        pos1 = true;
//                    }
//
//                    else if(x > pos2Left_X && x < pos2Right_X ||
//                            ADJUSTED_X > pos2Left_X && ADJUSTED_X < pos2Right_X)
//                    {
//                        pos2 = true;
//                    }
//                    else {
//                        pos3 = true;
//                    }
                }

            }
            contours.clear();
            skystoneIsCentered = found;
            found1 = pos1;
            found2 = pos2;
            found3 = pos3;
        }
        return thresholded;
    }

    public double getSkystoneCameraXPos() {
        return skystoneXPos;
    }
    public double getSkystoneCameraYPos() {
        return skystoneYPos;
    }

//    public int getSkystonePosition() {
//        if(found1 = true) {
//            return 1;
//        }
//        else if(found2 = true) {
//            return 2;
//        }
//        else if(found3 = true) {
//            return 3;
//        }
//        else { return 0; }
//    }

    public int getWidth() {
        return width;
    }
    public int getHeight() {
        return height;
    }

    public boolean getStoneCentered()  {
        return skystoneIsCentered;
    }

    public void setSkystoneFalse() {
        skystoneIsCentered = false;
    }

    public int getContoursFound() {
        return contoursFound;
    }


    public boolean isFound1() {
        return found1;
    }

    public boolean isFound2() {
        return found2;
    }

    public boolean isFound3() {
        return found3;
    }
}
