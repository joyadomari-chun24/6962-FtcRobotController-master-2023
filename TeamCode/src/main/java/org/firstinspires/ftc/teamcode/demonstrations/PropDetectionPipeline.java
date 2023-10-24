//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Canvas;
//
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Rect;
//import org.opencv.imgproc.Imgproc;
//
//public class PropDetectionPipeline extends VisionProcessor
//{
//    MatOfPoint largestContour;
//    @Override
//    public void init(int width, int height, CameraCalibration calibration)
//    {
//        //lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
//    }
//
//    @Override
//    public Object processFrame(Mat frame, long captureTimeNanos)
//    {
//        //
//    }
//
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        //creates a box with the largest contour at the center
//        if (largestContour != null) {
//            Rect rect = Imgproc.boundingRect(largestContour);
//
//            float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};
//        }
//    }
//}
