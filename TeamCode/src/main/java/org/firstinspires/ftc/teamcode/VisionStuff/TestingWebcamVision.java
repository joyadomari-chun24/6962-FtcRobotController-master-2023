package org.firstinspires.ftc.teamcode.VisionStuff;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous
public class TestingWebcamVision extends LinearOpMode {
    VisionProcessor visionProcessorTest = new PropDetectionProcessor();
    public static class CameraStreamProcessor implements CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        //Something to do with dashboard
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Creates an instance of the processor above
        //I don't understand how the CameraStreamProcessor and VisionProcessor work together yet
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        //Builds a vision portal with the processor we just made and sets the webcam (if we use .addProcessors, we can add multiple processors to the frame. See double vision example)
        new VisionPortal.Builder()
                .addProcessor(visionProcessorTest)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        //Sends the camera feed to the dashboard
        FtcDashboard.getInstance().startCameraStream(processor, 0);

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("LargestContourX: ", visionProcessorTest.largestContourX);
//            telemetry.addData("LargestContourY: ", visionProcessorTest.largestContourY);
//            telemetry.addLine(GetPropLocation(visionProcessorTest.largestContourX, visionProcessorTest.largestContourY));
            telemetry.update();

            sleep(100L);
        }
    }
}
