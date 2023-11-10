package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.ScoringStuff.ClawSubsystem;
@Autonomous
public class TrueAutoOp extends OpModeBase
{
    private Servo claw;
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

        Pose2d startPose = new Pose2d(12, -66, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory leftTrajectory = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        Trajectory middleTrajectory = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        Trajectory rightTrajectory = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        Trajectory scoreYellowPixel = drive.trajectoryBuilder(middleTrajectory.end())
                .lineToLinearHeading(new Pose2d(-20, -100, Math.toRadians(180)))
                        .build();

        Trajectory parkScore = drive.trajectoryBuilder(scoreYellowPixel.end())
                .lineToLinearHeading(new Pose2d(-30, -120, Math.toRadians(270)))
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
            drive.followTrajectory(scoreYellowPixel);
            drive.followTrajectory(parkScore);
            claw.setPosition(0.4);
        }

        else if (propLocation.equals("CENTER"))
        {
            drive.followTrajectory(middleTrajectory);
            drive.followTrajectory(scoreYellowPixel);
            drive.followTrajectory(parkScore);
            claw.setPosition(0.4);
        }

        else
        {
            drive.followTrajectory(rightTrajectory);
            drive.followTrajectory(scoreYellowPixel);
            drive.followTrajectory(parkScore);
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
