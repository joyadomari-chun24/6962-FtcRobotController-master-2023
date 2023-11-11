package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.config.Config;
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
@Config
@Autonomous
public class TrueAutoOp extends OpModeBase
{
    String propLocation;
    PropDetectionProcessor processor = new PropDetectionProcessor(true);

    //Middle coordinates
    public static int centerPurpleForward = 27; //good
    public static int centerYellowX = 50;
    public static int centerYellowY = -40;

    //Left coordinates
    public static int leftPurpleX = 27;
    public static int leftPurpleY = 27;
    public static int leftYellowX = 50; //good
    public static int leftYellowY = -40; //good

    //Right coordinates
    public static int rightPurpleX = 27;
    public static int rightPurpleY = 27;
    public static int rightYellowX = 50; //good
    public static int rightYellowY = -40;

    //Parking coordinates
    public static int parkX = 56;
    public static int parkY = -14;
    public static int parkTan = 0;
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
                .splineTo(new Vector2d(leftPurpleX, leftPurpleY), Math.toRadians(0))
                .build();

        Trajectory middlePurpleScore = drive.trajectoryBuilder(startPose)
                .forward(centerPurpleForward)
                .build();

        Trajectory rightPurpleScore = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(rightPurpleX, rightPurpleY), Math.toRadians(0))
                .build();

        //Scoring yellow pixel
        Trajectory leftYellowScore = drive.trajectoryBuilder(leftPurpleScore.end())
                .lineToLinearHeading(new Pose2d(leftYellowX, leftYellowY, Math.toRadians(0)))
                        .build();

        Trajectory middleYellowScore = drive.trajectoryBuilder(middlePurpleScore.end())
                .lineToLinearHeading(new Pose2d(centerYellowX, centerYellowY, Math.toRadians(0)))
                .build();

        Trajectory rightYellowScore = drive.trajectoryBuilder(rightPurpleScore.end())
                .lineToLinearHeading(new Pose2d(rightYellowX, rightYellowY, Math.toRadians(0)))
                .build();

        //Parking
        Trajectory parkBackup = drive.trajectoryBuilder(leftYellowScore.end())
                .back(24)
                .build();
        Trajectory parkScore = drive.trajectoryBuilder(leftYellowScore.end())
                .splineToSplineHeading(new Pose2d(parkX, parkY, Math.toRadians(0)), Math.toRadians(parkTan))
                .build();

//        Trajectory parkStill = drive.trajectoryBuilder(leftYellowScore.end())
//                .build();

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
            schedule(new SequentialCommandGroup(
                    claw.closeClaw(),
                    new InstantCommand(() -> drive.followTrajectory(leftPurpleScore)),
                    arm.deployFront(),
                    new InstantCommand(() -> drive.followTrajectory(leftYellowScore)),
                    claw.openClaw(),
                    new InstantCommand(() -> drive.followTrajectory(parkBackup)),
                    new InstantCommand(() -> drive.followTrajectory(parkScore)),
                    arm.deployBack() //I'm assuming deployBack is temporary
            ));
        }
        else if (propLocation.equals("CENTER"))
        {
            //Using the scheduler allows us to run commands in auto
            schedule(new SequentialCommandGroup(
                    claw.closeClaw(),
                    new InstantCommand(() -> drive.followTrajectory(middlePurpleScore)),
                    arm.deployFront(),
                    new InstantCommand(() -> drive.followTrajectory(middleYellowScore)),
                    claw.openClaw(),
                    new InstantCommand(() -> drive.followTrajectory(parkBackup)),
                    new InstantCommand(() -> drive.followTrajectory(parkScore)),
                    arm.deployBack() //I'm assuming deployBack is temporary
            ));
        }
        else
        {
            schedule(new SequentialCommandGroup(
                    claw.closeClaw(),
                    new InstantCommand(() -> drive.followTrajectory(rightPurpleScore)),
                    arm.deployFront(),
                    new InstantCommand(() -> drive.followTrajectory(rightYellowScore)),
                    claw.openClaw(),
                    new InstantCommand(() -> drive.followTrajectory(parkBackup)),
                    new InstantCommand(() -> drive.followTrajectory(parkScore)),
                    arm.deployFront() //I'm assuming deployBack is temporary
            ));
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
