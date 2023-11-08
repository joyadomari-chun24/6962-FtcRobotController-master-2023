package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class TrueAutoOp extends OpModeBase
{
    String propLocation;
    PropDetectionProcessor processor = new PropDetectionProcessor(true);
    @Override
    public void initialize()
    {
        super.initialize();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(1,1), Math.toRadians(45))
                .build();

        Trajectory middleTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .strafeRight(2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        propLocation = processor.GetPropLocation();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        if(propLocation.equals("LEFT"))
        {
            drive.followTrajectory(leftTrajectory);
        }

        else if (propLocation.equals("MIDDLE"))
        {
            drive.followTrajectory(middleTrajectory);
        }

        else
        {
            drive.followTrajectory(rightTrajectory);
        }
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
        telemetry.addData("LargestContourX: ", processor.GetContourX());
        telemetry.addData("LargestContourY: ", processor.GetContourY());
        telemetry.addLine(processor.GetPropLocation());
        telemetry.update();
    }
}
