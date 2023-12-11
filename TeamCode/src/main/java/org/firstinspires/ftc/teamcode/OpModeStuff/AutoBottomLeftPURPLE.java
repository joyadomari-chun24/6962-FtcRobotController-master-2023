package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "Active Autos")
public class AutoBottomLeftPURPLE extends OpModeBase
{
    String propLocation;

    public static double offset = 6;

    //Middle coordinates
    public static int purpleX = -36;
    public static int purpleY = 35;
    public static int middlePurpleX = -36;
    public static int middlePurpleY = 16;
    public static int trussBottomX = -36;
    public static int trussBottomY = 11;
    public static int trussTopX = 13;
    public static int trussTopY = 0;
    public static double centerPurpleForward = 30 * 2.5;
    public static int centerYellowX = 56;
    public static int centerYellowY = 29;

    //Left coordinates
    public static int leftPurpleX = -24;
    public static int leftPurpleY = 40;
    public static int leftYellowX = 56;
    public static int leftYellowY = 25;

    //Right coordinates
    public static int rightPurpleX = -43;
    public static int rightPurpleY = 33;
    public static int rightYellowX = 56;
    public static int rightYellowY = 15;
    public static int rightBackup = 5;

    //Parking coordinates
    public static int parkX = 56;
    public static int parkY = 17;
    @Override
    public void initialize()
    {
        colorProcessor = new PropDetectionProcessor(true);
        super.initialize();

        Pose2d startPose = new Pose2d(-36.5, 64, Math.toRadians(-180));

        roadrunnerMecanumDrive.setPoseEstimate(startPose);

        //Scoring purple pixel
        TrajectorySequence leftPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence leftPurpleBackup = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPurpleScore.end())
                .waitSeconds(2)
                .back(4)
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurplePrep = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(90)))
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePurplePrep.end())
                .strafeLeft(22)
                .waitSeconds(1)
                .build();

        TrajectorySequence rightPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(purpleX, purpleY))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightPurpleBackup = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPurpleScore.end())
                .waitSeconds(2)
                .back(4)
                .waitSeconds(1)
                .build();

        //truss
        TrajectorySequence leftBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPurpleBackup.end())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(90)))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPurpleBackup.end())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(90)))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .build();

        TrajectorySequence middleBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePurpleScore.end())
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(trussBottomX, trussBottomY))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .build();

        TrajectorySequence leftTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftBottomTruss.end())
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence middleTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleBottomTruss.end())
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightBottomTruss.end())
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        //Move in
        Trajectory goToMiddle = roadrunnerMecanumDrive.trajectoryBuilder(middlePurpleScore.end())
                .lineToLinearHeading(new Pose2d(-36, 0, Math.toRadians(0)))
                .build();


        //Scoring yellow pixel
        TrajectorySequence leftYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftTopTruss.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(leftYellowX, leftYellowY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence middleYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleTopTruss.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(centerYellowX, centerYellowY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        //Trajectory rightPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
        //.back(rightBackup)
        //.build();
        TrajectorySequence rightYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightTopTruss.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(rightYellowX, rightYellowY, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        //Trajectory middlePostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
        //.back(rightBackup)
        //.build();

        //Trajectory leftPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
        //.back(rightBackup)
        //.build();

        //Trajectory trussPath = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
        //.lineToLinearHeading(new Pose2d(0, 0))
        //.build();
        //Parking
        TrajectorySequence leftPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftYellowScore.end())
                .waitSeconds(2)
                .back(6)
                .turn(Math.toRadians(-90-offset))
                .waitSeconds(1)
                .forward(24)
                .strafeLeft(10)
                .build();

        TrajectorySequence middlePark = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleYellowScore.end())
                .waitSeconds(2)
                .back(6)
                .turn(Math.toRadians(-90-offset))
                .waitSeconds(1)
                .forward(24)
                .strafeLeft(10)
                .build();

        TrajectorySequence rightPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightYellowScore.end())
                .waitSeconds(2)
                .back(6)
                .turn(Math.toRadians(-90-offset))
                .waitSeconds(1)
                .forward(24)
                .strafeLeft(10)
                .build();

        //Trajectory parkBackup = roadrunnerMecanumDrive.trajectoryBuilder(trussPath.end())
        //.back(22)
        //.build();

        //Trajectory parkScore = roadrunnerMecanumDrive.trajectoryBuilder(trussPath.end())
        //.lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(0)))
        //.build();

        while(!isStarted())
        {
            /*
             * The camera won't immediately detect a largest contour after being turned on, so it
             * will be (0,0) for a bit, which falls in the LEFT range. After init, wait a few seconds
             * before starting the OpMode so that it detects the right positon.
             */
            propLocation = colorProcessor.GetPropLocation();
            telemetry.addData("Prop Location: ", propLocation);
            telemetry.addData("Camera state ", aprilPortal.getCameraState());
            telemetry.update();
        }

        if(isStopRequested()) return;

        //Turns off camera
        if (colorPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            colorPortal.stopLiveView();
            colorPortal.stopStreaming();
        }

        if(propLocation.equals("LEFT"))
        {
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPurpleScore)),
                    clawL.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPurpleBackup)),
                    arm.pickupFront()
            ));
        }
        else if (propLocation.equals("CENTER"))
        {
            //Using the scheduler allows us to run commands in auto
        }
        else
        {
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightPurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightPurpleBackup)),
                    arm.pickupFront()
            ));
        }

        //sleep(23000);

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
        telemetry.addData("LargestContourX: ", colorProcessor.GetContourX());
        telemetry.addData("LargestContourY: ", colorProcessor.GetContourY());
        telemetry.addLine(colorProcessor.GetPropLocation());
        telemetry.update();
    }
}
