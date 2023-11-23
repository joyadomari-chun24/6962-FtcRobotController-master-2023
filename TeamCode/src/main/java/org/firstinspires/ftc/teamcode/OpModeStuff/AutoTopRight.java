package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.trajectory.constraint.MecanumDriveKinematicsConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(preselectTeleOp = "RedTeleOp", group = "Active Autos")
public class AutoTopRight extends OpModeBase
{
    //I'm also using this one for experimental testing right now
    String propLocation;
    PropDetectionProcessor processor = new PropDetectionProcessor(false);

    //Middle coordinates
    public static int centerPurpleForward = 30;
    public static int centerYellowX = 53;
    public static int centerYellowY = -36;

    //New center!
    public static int otherPurpleforward = 35;

    //Left coordinates
    public static int leftPurpleX = -3;
    public static int leftPurpleY = -40;
    public static int leftYellowX = 54;
    public static int leftYellowY = -31;

    //Right coordinates
    public static int rightPurpleX = 33;
    public static int rightPurpleY = -43;
    public static int rightYellowX = 53;
    public static int rightYellowY = -42;
    public static int rightBackup = 5;

    //Experimental coordinates
    public static int purpleX = 13;
    public static int purpleY = -36;

    //Parking coordinates
    public static int parkX = 56;
    public static int parkY = -17;
    @Override
    public void initialize()
    {
        super.initialize();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        Pose2d startPose = new Pose2d(9, -63.5, Math.toRadians(0));

        roadrunnerMecanumDrive.setPoseEstimate(startPose);

        //Scoring purple pixel
        Trajectory leftPurpleScore = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(150)))
                .build();

        Trajectory middlePurpleScore = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(90)))
                .build();


        //Trajectory leftPurpleScore = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                //.lineTo(new Vector2d(leftPurpleX, leftPurpleY))
                //.build();

        //Trajectory middlePurpleScore = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                //.forward(centerPurpleForward)
                //.build();

        //Trajectory rightPurpleScore = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                //.lineTo(new Vector2d(rightPurpleX, rightPurpleY))
                //.build();

        //Trajectory rightPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
                //.back(rightBackup)
                //.build();

        Trajectory middlePostPurple = roadrunnerMecanumDrive.trajectoryBuilder(leftPurpleScore.end())
                .back(rightBackup)
                .build();

        //Trajectory leftPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
                //.back(rightBackup)
                //.build();

        //New center!
        Trajectory otherPurple = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                .forward(otherPurpleforward)
                .build();
        Trajectory rightBack = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                .back(5)
                .build();

        //Scoring yellow pixel
        Trajectory leftYellowScore = roadrunnerMecanumDrive.trajectoryBuilder(leftPurpleScore.end())
                .lineToLinearHeading(new Pose2d(leftYellowX, leftYellowY, Math.toRadians(0)))
                .build();

        Trajectory middleYellowScore = roadrunnerMecanumDrive.trajectoryBuilder(middlePurpleScore.end())
                .lineToLinearHeading(new Pose2d(centerYellowX, centerYellowY, Math.toRadians(0)))
                .build();

        Trajectory rightYellowScore = roadrunnerMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightYellowX, rightYellowY, Math.toRadians(0)))
                .build();

        //right side scoring
        Trajectory rightPurpleScore = roadrunnerMecanumDrive.trajectoryBuilder(rightYellowScore.end())
                .lineToLinearHeading(new Pose2d(rightPurpleX, rightPurpleY, Math.toRadians(0)))
                .build();

        //Parking
        Trajectory parkLeft = roadrunnerMecanumDrive.trajectoryBuilder(leftYellowScore.end())
                .back(2)
                .build();

        Trajectory parkMiddle = roadrunnerMecanumDrive.trajectoryBuilder(middleYellowScore.end())
                .back(2)
                .build();

        Trajectory parkRight = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
                .lineToLinearHeading(new Pose2d(centerYellowX, centerYellowY, Math.toRadians(180)))
                .build();

        //Trajectory parkScore = roadrunnerMecanumDrive.trajectoryBuilder(parkBackup.end())
                //.lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(0)))
                //.build();

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

            //aprilPortal.resumeStreaming();
        }

        if(propLocation.equals("LEFT"))
        {
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-90))),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkLeft))
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                   // arm.pickupFront()
            ));
        }
        else if (propLocation.equals("CENTER"))
        {
            //Using the scheduler allows us to run commands in auto
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-90))),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(middlePurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(middleYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkMiddle))
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    //arm.pickupFront()
            ));
        }
        else
        {
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    arm.deployFront(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-90))),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(rightYellowScore)),
                    clawR.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkRight)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(rightPurpleScore)),
                    arm.pickupFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-190))),
                    clawL.openClaw(),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkRight))
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    //arm.deployFront(),
                    //clawR.openClaw()
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    //arm.pickupFront()
            ));
        }

//        if (aprilPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
//        {
//            aprilPortal.stopLiveView();
//            aprilPortal.stopStreaming();
//        }
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
