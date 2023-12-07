package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "Active Autos")
public class AutoTopLeft extends OpModeBase
{
    String propLocation;
    PropDetectionProcessor processor = new PropDetectionProcessor(true);

    //Experimental coordinates
    public static int purpleX = 13;
    public static int purpleY = 39;
    public static double offset = 6;

    //Middle coordinates
    public static int centerPurpleForward = 30;
    public static int centerYellowX = 54;
    public static int centerYellowY = 37;

    //Left coordinates
    public static int leftPurpleX = 29;
    public static int leftPurpleY = 35;
    public static int leftYellowX = 54;
    public static int leftYellowY = 40;

    //Right coordinates
    public static int rightPurpleX = 10;
    public static int rightPurpleY = 33;
    public static int rightYellowX = 54;
    public static int rightYellowY = 30;
    public static int rightBackup = 5;

    //Parking coordinates
    public static int parkX = 56;
    public static int parkY = 17;

    //truss coordinates
    public static int trussTopX = 26;
    public static int trussTopY = 11;
    public static int trussBottomX = -36;
    public static int trussBottomY = 15;
    @Override
    public void initialize()
    {
        super.initialize();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "colorCam"))
                .build();

        Pose2d startPose = new Pose2d(14.5, 63.5, Math.toRadians(-180));

        roadrunnerMecanumDrive.setPoseEstimate(startPose);

        //Scoring purple pixel
        TrajectorySequence middlePurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(-90+offset)))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(-150+offset)))
                .waitSeconds(1)
                .build();

        Trajectory middlePostPurple = roadrunnerMecanumDrive.trajectoryBuilder(middlePurpleScore.end())
                .back(rightBackup)
                .build();

        Trajectory leftPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
                .back(rightBackup)
                .build();


        //Scoring yellow pixel
        TrajectorySequence leftYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(leftYellowX, leftYellowY, Math.toRadians(0+offset)))
                .waitSeconds(2)
                .build();

        TrajectorySequence leftPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftYellowScore.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(leftPurpleX, leftPurpleY, Math.toRadians(0+offset)))
                .turn(Math.toRadians(190))
                .waitSeconds(2)
                .build();


        TrajectorySequence middleYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePurpleScore.end())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(centerYellowX, centerYellowY, Math.toRadians(0+offset)))
                .waitSeconds(1)
                .build();

        Trajectory rightPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
                .back(rightBackup)
                .build();

        TrajectorySequence rightYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPurpleScore.end())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(rightYellowX, rightYellowY, Math.toRadians(0+offset)))
                .waitSeconds(1)
                .build();

        //Parking
        //Trajectory parkBackup = roadrunnerMecanumDrive.trajectoryBuilder(leftYellowScore.end())
        //.back(22)
        //.build();

        //Trajectory parkScore = roadrunnerMecanumDrive.trajectoryBuilder(leftYellowScore.end())
        //.lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(0)))
        //.build();

        TrajectorySequence rightPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightYellowScore.end())
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

        TrajectorySequence leftPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPurpleScore.end())
                .waitSeconds(2)
                .back(6)
                .turn(Math.toRadians(90+offset))
                .waitSeconds(1)
                .forward(24)
                .strafeLeft(20)
                .build();

        //truss
        TrajectorySequence rightTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPark.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(185)))
                .build();

        TrajectorySequence middleTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePark.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(185)))
                .build();

        TrajectorySequence leftTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPark.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(185)))
                .build();

        TrajectorySequence rightBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightTopTruss.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(-180)))
                .build();

        TrajectorySequence middleBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleTopTruss.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(-180)))
                .build();

        TrajectorySequence leftBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftTopTruss.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(-180)))
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
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    arm.deployFront(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftYellowScore)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(middlePurpleScore)),
                    //arm.pickupFront(),
                    clawR.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    //arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    //arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPark)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftTopTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftBottomTruss)),
                    arm.pickupFront()
            ));
        }
        else if (propLocation.equals("CENTER"))
        {
            //Using the scheduler allows us to run commands in auto
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middlePurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middleYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middlePark)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middleTopTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middleBottomTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    arm.pickupFront()
            ));
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
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightPark)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightTopTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightBottomTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    arm.pickupFront()
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
