package org.firstinspires.ftc.teamcode.drive;

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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class VisionPortalStreamingOpMode extends LinearOpMode {
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        static final Rect LEFT_TESTING_RECTANGLE = new Rect(
                new Point(0, 0),
                new Point(0, 0));

        static final Rect RIGHT_TESTING_RECTANGLE = new Rect(
                new Point(0, 0),
                new Point(0, 0));


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

            if (imgMat.empty())
                return null;

            Scalar lowRedHSV = new Scalar(0, 70, 50);
            Scalar highRedHSV = new Scalar(10, 255, 255);

            Mat imgThresh = new Mat();

            //Makes the imgMat a balck and white image of red objects
            Core.inRange(imgMat, lowRedHSV, highRedHSV, imgThresh);

            Mat imgMask = new Mat();

            //colors back in the filtered area
            Core.bitwise_and(imgMat, imgMat, imgMask, imgThresh);

            //Calculate average HSV value of the colors
            Scalar average = Core.mean(imgMask, imgThresh);

            //Scales mask to arbitrary saturation of 150 (from tutorial)
            Mat scaledMask = new Mat();
            imgMask.convertTo(scaledMask, -1, 150/average.val[1], 0);

            Scalar strictLowRedHSV = new Scalar(0, 150, 50);
            Scalar strictHighRedHSV = new Scalar(10, 255, 255);

            Mat scaledThresh = new Mat();

            Core.inRange(scaledMask, strictLowRedHSV, strictHighRedHSV, scaledThresh);

            //Colors in the scaled objects
            Mat finalMask = new Mat();
            Core.bitwise_and(imgMat, imgMat, finalMask, scaledThresh);

            //Displays edges
            Mat imgEdges = new Mat();
            Imgproc.Canny(scaledThresh, imgEdges, 100, 200);

            //Processes edges somehow (this part is a little mysterious)?
            List<MatOfPoint> imgContours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(scaledThresh, imgContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //Release and return data
            frame.release();
            scaledThresh.copyTo(frame);
            Imgproc.cvtColor(finalMask, frame, Imgproc.COLOR_HSV2RGB);
            scaledThresh.release();
            scaledMask.release();
            imgMat.release();
            imgMask.release();
            imgEdges.release();
            imgThresh.release();
            finalMask.release();

            //Not sure what this does exactly. Removing it makes the camera a black screen on dashboard
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
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
            sleep(100L);
        }
    }
}
