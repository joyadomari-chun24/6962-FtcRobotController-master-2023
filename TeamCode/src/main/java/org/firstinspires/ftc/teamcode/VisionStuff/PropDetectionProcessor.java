package org.firstinspires.ftc.teamcode.VisionStuff;

import android.graphics.Camera;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.CameraState;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionProcessor implements VisionProcessor
{
    //private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    double largestContourArea;
    MatOfPoint largestContour;
    double minArea;
    double largestContourX;
    double largestContourY;
    boolean propIsBlue = false;
    Scalar lowRedHSV = new Scalar(0, 70, 50);
    Scalar highRedHSV = new Scalar(10, 255, 255);
    Scalar strictLowRedHSV = new Scalar(0, 150, 50);
    Scalar strictHighRedHSV = new Scalar(10, 255, 255);
    Scalar lowBlueHSV = new Scalar(100, 70, 0);
    Scalar highBlueHSV = new Scalar(140, 255, 255);
    Scalar strictLowBlueHSV = new Scalar(100, 150, 0);
    Scalar strictHighBlueHSV = new Scalar(140, 255, 255);
//    Scalar highHSV = propIsBlue ? highBlueHSV : highRedHSV;
//    Scalar lowHSV = propIsBlue ? lowBlueHSV : lowRedHSV;
//    Scalar strictHighHSV = propIsBlue ? strictHighBlueHSV : strictHighRedHSV;
//    Scalar strictLowHSV = propIsBlue ? strictLowBlueHSV : strictLowRedHSV;
    Scalar highHSV;
    Scalar lowHSV;
    Scalar strictHighHSV;
    Scalar strictLowHSV;
    double leftZone = 150;
    double rightZone = 500;

    public PropDetectionProcessor(boolean propIsBlue)
    {
        this.propIsBlue = propIsBlue;
        highHSV = propIsBlue ? highBlueHSV : highRedHSV;
        lowHSV = propIsBlue ? lowBlueHSV : lowRedHSV;
        strictHighHSV = propIsBlue ? strictHighBlueHSV : strictHighRedHSV;
        strictLowHSV = propIsBlue ? strictLowBlueHSV : strictLowRedHSV;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        //lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        Mat imgMat = new Mat();

        //Converts from RGB to HSV
        Imgproc.cvtColor(frame, imgMat, Imgproc.COLOR_RGB2HSV);

        //Cuts down on loop times I guess by terminating loop early if there's nothing there anyway
        if (imgMat.empty())
            return null;

        //Makes the imgMat a black and white image of only red
        Mat imgThresh = new Mat();
        Core.inRange(imgMat, lowHSV, highHSV, imgThresh);

        //Color back in the filtered area with RGB
        Mat imgMask = new Mat();
        Core.bitwise_and(imgMat, imgMat, imgMask, imgThresh);

        //Calculate average HSV value of the colors
        Scalar average = Core.mean(imgMask, imgThresh);

        //Scales the mask to an arbitrary saturation of 150 so everything's the same saturation (from tutorial)
        Mat scaledMask = new Mat();
        imgMask.convertTo(scaledMask, -1, 150/average.val[1], 0);

        //Creates new black and white mat with a stricter threshold
        Mat scaledThresh = new Mat();
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        //Colors back in the colors
        Mat finalMask = new Mat();
        Core.bitwise_and(imgMat, imgMat, finalMask, scaledThresh);

        //Process the edges
        List<MatOfPoint> imgContours = new ArrayList<>();
        imgContours.clear();
        Mat hierarchy = new Mat();
        Imgproc.findContours(scaledThresh, imgContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        largestContourArea = -1;

        //Goes through every contour to find the largest one
        for (MatOfPoint contour : imgContours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea && area > minArea) {
                largestContour = contour;
                largestContourArea = area;
            }
        }

        largestContourX = largestContourY = -1;

        //Not exactly sure what moments are, but this finds the x and y of the contour
        if (largestContour != null) {
            Moments moment = Imgproc.moments(largestContour);
            largestContourX = (moment.m10 / moment.m00);
            largestContourY = (moment.m01 / moment.m00);
        }

        //Release and return data (release as much as you can)
        frame.release();
        scaledThresh.copyTo(frame);
        Imgproc.cvtColor(finalMask, frame, Imgproc.COLOR_HSV2RGB);
        scaledThresh.release();
        scaledMask.release();
        imgMat.release();
        imgMask.release();
        imgThresh.release();
        finalMask.release();

        //Sends bitmap to be displayed on the dashboard
//        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
//        Utils.matToBitmap(frame, b);
//        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        // do nothing
    }

    public String GetPropLocation()
    {
        if(largestContourX < leftZone)
            return "LEFT";
        else if (largestContourX > rightZone)
            return "RIGHT";
        else
            return "CENTER";
    }

    public double GetContourX()
    {
        return largestContourX;
    }

    public double GetContourY()
    {
        return largestContourY;
    }

}
