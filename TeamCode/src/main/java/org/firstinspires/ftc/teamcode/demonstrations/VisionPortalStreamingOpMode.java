package org.firstinspires.ftc.teamcode.demonstrations;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import android.graphics.Paint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous(name = "Dashboard Vision", group = "tests")
public class VisionPortalStreamingOpMode extends LinearOpMode {
    //Zones to dertermine the prop's position on the 640x400 image
    double leftZone = 100;
    double rightZone = 500;
    public static boolean propIsBlue = false; //Made public static to be changed by dashboard

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        double largestContourArea;
        MatOfPoint largestContour;
        double minArea;
        public double largestContourX;
        public double largestContourY;
        Paint propLines = new Paint();

        Scalar lowRedHSV = new Scalar(0, 70, 50);
        Scalar highRedHSV = new Scalar(10, 255, 255);
        Scalar strictLowRedHSV = new Scalar(0, 150, 50);
        Scalar strictHighRedHSV = new Scalar(10, 255, 255);
        Scalar lowBlueHSV = new Scalar(100, 70, 0);
        Scalar highBlueHSV = new Scalar(140, 255, 255);
        Scalar strictLowBlueHSV = new Scalar(100, 150, 0);
        Scalar strictHighBlueHSV = new Scalar(140, 255, 255);
        Scalar highHSV = propIsBlue ? highBlueHSV : highRedHSV;
        Scalar lowHSV = propIsBlue ? lowBlueHSV : lowRedHSV;
        Scalar strictHighHSV = propIsBlue ? strictHighBlueHSV : strictHighRedHSV;
        Scalar strictLowHSV = propIsBlue ? strictLowBlueHSV : strictLowRedHSV;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        //This is what we want to do to the frame
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
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
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            //creates a box with the largest contour at the center
            if (largestContour != null) {
                Rect rect = Imgproc.boundingRect(largestContour);

                float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};
            }
        }

        //Something to do with dashboard
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
    //Returns the prop location
    public String GetPropLocation(double contourX, double contourY)
    {
        if(contourX < leftZone)
            return "Left";
        else if (contourX > rightZone)
            return "Right";
        else
            return "Center (or none I haven't figured that out)";
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Creates an instance of the processor above
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        //Builds a vision portal with the processor we just made and sets the webcam (if we use .addProcessors, we can add multiple processors to the frame. See double vision example)
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        //Sends the camera feed to the dashboard
        FtcDashboard.getInstance().startCameraStream(processor, 0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("LargestContourX: ", processor.largestContourX);
            telemetry.addData("LargestContourY: ", processor.largestContourY);
            telemetry.addLine(GetPropLocation(processor.largestContourX, processor.largestContourY));
            telemetry.update();

            sleep(100L);
        }
    }
}
