package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

        Pose2d startPose = new Pose2d(12, -66, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //Scoring purple pixel
        Trajectory leftPurpleScore = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        Trajectory middlePurpleScore = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        Trajectory rightPurpleScore = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        //Scoring yellow pixel
        Trajectory leftYellowScore = drive.trajectoryBuilder(leftPurpleScore.end())
                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(0)))
                        .build();

        Trajectory middleYellowScore = drive.trajectoryBuilder(middlePurpleScore.end())
                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(0)))
                .build();

        Trajectory rightYellowScore = drive.trajectoryBuilder(rightPurpleScore.end())
                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(0)))
                .build();

        //Parking
        Trajectory parkScore = drive.trajectoryBuilder(leftYellowScore.end())
                .lineToLinearHeading(new Pose2d(43, -46, Math.toRadians(0)))
                .build();

        Trajectory parkStill = drive.trajectoryBuilder(leftYellowScore.end())
                .lineToLinearHeading(new Pose2d(leftYellowScore.end().getX(), leftYellowScore.end().getY(), Math.toRadians(0)))
                .build();

        while(!isStarted())
        {
            /*
             * The camera won't immediately detect a largest contour after being turned on, so it
             * will be (0,0) for a bit, which falls in the LEFT range. After init, wait a few seconds
             * before starting the OpMode so that it detects the right positon.
             */
            propLocation = processor.GetPropLocation();
            telemetry.addData("Prop Location: ", propLocation);
            telemetry.update();
        }

        if(isStopRequested()) return;

        //Turns off camera
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        if(propLocation.equals("LEFT"))
        {
            drive.followTrajectory(leftPurpleScore);
            drive.followTrajectory(leftYellowScore);
            drive.followTrajectory(parkScore);
            //claw.setPosition(0.4);
        }
        else if (propLocation.equals("CENTER"))
        {
            //Using the scheduler allows us to run commands in auto
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> drive.followTrajectory(middlePurpleScore)),
                    arm.deployFront(),
                    new InstantCommand(() -> drive.followTrajectory(leftYellowScore)),
                    claw.openClaw(),
                    new InstantCommand(() -> drive.followTrajectory(parkScore))
            ));
        }
        else
        {
            drive.followTrajectory(rightPurpleScore);
            drive.followTrajectory(leftYellowScore);
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
