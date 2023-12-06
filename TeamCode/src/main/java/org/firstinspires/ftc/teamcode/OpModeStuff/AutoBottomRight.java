package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(preselectTeleOp = "RedTeleOp", group = "Active Autos")
public class AutoBottomRight extends OpModeBase
{
    String propLocation;
    PropDetectionProcessor processor = new PropDetectionProcessor(false);

    //Experimental coordinates
    public static int PurpleX = -36;
    public static int PurpleY = -36;
    public static int middlePurpleX = -36;
    public static int middlePurpleY = -16;
    public static int trussBottomX = -36;
    public static int trussBottomY = -11;
    public static int trussTopX = 13;
    public static int trussTopY = -11;

    //Middle coordinates
    public static double centerPurpleForward = 30 * 2.5;
    public static int middleYellowX = 56;
    public static int middleYellowY = -39;

    //New center!
    public static int otherPurpleforward = 22;

    //Left coordinates
    public static int leftPurpleX = -43;
    public static int leftPurpleY = -40;
    public static int leftYellowX = 57;
    public static int leftYellowY = -35;

    //Right coordinates
    public static int rightPurpleX = -24;
    public static int rightPurpleY = -33;
    public static int rightYellowX = 56;
    public static int rightYellowY = -42;
    public static int rightBackup = 5;

    //Parking coordinates
    public static int parkX = 56;
    public static int parkY = -17;

    @Override
    public void initialize()
    {
        super.initialize();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "colorCam"))
                .build();

        Pose2d startPose = new Pose2d(-39, -63.5, Math.toRadians(0));

        roadrunnerMecanumDrive.setPoseEstimate(startPose);

        //Scoring purple pixel
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

        //Trajectory middlePostPurple = roadrunnerMecanumDrive.trajectoryBuilder(leftPurpleScore.end())
        //.back(rightBackup)
        //.build();

        //Trajectory leftPostPurple = roadrunnerMecanumDrive.trajectoryBuilder(leftPurpleScore.end())
        //.back(rightBackup)
        //.build();

        TrajectorySequence leftPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PurpleX, PurpleY, Math.toRadians(-179)))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(PurpleX, PurpleY))
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurplePrep = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PurpleX, PurpleY, Math.toRadians(-90)))
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePurplePrep.end())
                .lineToConstantHeading(new Vector2d(middlePurpleX, middlePurpleY))
                .waitSeconds(1)
                .build();

        TrajectorySequence leftBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPurpleScore.end())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(-90)))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPurpleScore.end())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(trussBottomX, trussBottomY, Math.toRadians(-90)))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .build();

        TrajectorySequence middleBottomTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePurpleScore.end())
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(trussBottomX, trussBottomY))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .build();

        TrajectorySequence leftTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftBottomTruss.end())
                .lineToConstantHeading(new Vector2d(trussTopX, trussTopY))
                .waitSeconds(1)
                .build();

        TrajectorySequence middleTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleBottomTruss.end())
                .lineToConstantHeading(new Vector2d(trussTopX, trussTopY))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightBottomTruss.end())
                .lineToConstantHeading(new Vector2d(trussTopX, trussTopY))
                .waitSeconds(1)
                .build();

        //New center!


        //Scoring yellow pixel
        TrajectorySequence leftYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftTopTruss.end())
                .lineToLinearHeading(new Pose2d(leftYellowX, leftYellowY, Math.toRadians(0)))
                .waitSeconds(2)
                .build();

        TrajectorySequence middleYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleTopTruss.end())
                .lineToLinearHeading(new Pose2d(middleYellowX, middleYellowY, Math.toRadians(0)))
                .waitSeconds(2)
                .build();

        TrajectorySequence rightYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightTopTruss.end())
                .lineToLinearHeading(new Pose2d(rightYellowX, rightYellowY, Math.toRadians(0)))
                .waitSeconds(2)
                .build();

        Trajectory trussPath = roadrunnerMecanumDrive.trajectoryBuilder(rightPurpleScore.end())
            .lineToLinearHeading(new Pose2d(0, 0))
            .build();

        //Parking
        Trajectory parkBackup = roadrunnerMecanumDrive.trajectoryBuilder(leftYellowScore.end())
                .back(22)
                .build();

        Trajectory parkScore = roadrunnerMecanumDrive.trajectoryBuilder(leftYellowScore.end())
                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(0)))
                .build();

        TrajectorySequence leftPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftYellowScore.end())
                .back(6)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .back(24)
                .strafeLeft(10)
                .build();

        TrajectorySequence middlePark = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleYellowScore.end())
                .back(6)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .back(24)
                .strafeLeft(10)
                .build();

        TrajectorySequence rightPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightYellowScore.end())
                .back(6)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .back(24)
                .strafeLeft(10)
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
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPurpleScore)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-181))),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftBottomTruss)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftTopTruss)),
                    //clawR.openClaw()
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    //arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPark))
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    //arm.pickupFront()
            ));
        }
        else if (propLocation.equals("CENTER"))
        {
            //Using the scheduler allows us to run commands in auto
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middlePurplePrep)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middlePurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middleBottomTruss)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middleTopTruss)),
                    //clawR.openClaw()
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    //arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middleYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(middlePark))
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    //arm.pickupFront()
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
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightBottomTruss)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightTopTruss)),
                    //clawR.openClaw()
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    //arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightPark))
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    //arm.pickupFront()
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
        telemetry.addData("LargestContourX: ", processor.GetContourX());
        telemetry.addData("LargestContourY: ", processor.GetContourY());
        telemetry.addLine(processor.GetPropLocation());
        telemetry.update();
    }
}
