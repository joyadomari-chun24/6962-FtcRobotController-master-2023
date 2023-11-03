package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class TrueAutoOp extends OpModeBase
{
    PropDetectionProcessor processor = new PropDetectionProcessor();
    @Override
    public void initialize()
    {
        super.initialize();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    @Override
    public void run()
    {
        super.run();
        roadrunnerMecanumDrive.update();
        Pose2d poseEstimate = roadrunnerMecanumDrive.getPoseEstimate();

        telemetry.addData("Gyro Heading", gyroManager.getHeading());
        telemetry.addData("x cord", poseEstimate.getX());
        telemetry.addData("y cord", poseEstimate.getY());
        telemetry.addData("roadrunner heading", poseEstimate.getHeading());
//        telemetry.addData("LargestContourX: ", processor.largestContourX);
//        telemetry.addData("LargestContourY: ", processor.largestContourY);
        telemetry.addLine(processor.GetPropLocation());
        telemetry.update();
    }
}
