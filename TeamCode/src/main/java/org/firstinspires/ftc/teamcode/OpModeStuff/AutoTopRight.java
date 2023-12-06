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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(preselectTeleOp = "RedTeleOp", group = "Active Autos")
public class AutoTopRight extends OpModeBase
{
    //I'm also using this one for experimental testing right now
    String propLocation;

    //https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AsyncFollowingFSM.java
    enum State
    {
        PLEFT, PCENTER, PRIGHT,
        YLEFT, YCENTER, YRIGHT,
        PARKING,
        IDLE
    }
    private State currentState = State.IDLE;

    //Middle coordinates
    public static int centerPurpleForward = 30;
    public static int centerYellowX = 56;
    public static int centerYellowY = -36;

    //New center!
    public static int otherPurpleforward = 35;

    //Left coordinates
    public static int leftPurpleX = -3;
    public static int leftPurpleY = -40;
    public static int leftYellowX = 56;
    public static int leftYellowY = -31;

    //Right coordinates
    public static int rightPurpleX = 33;
    public static int rightPurpleY = -43;
    public static int rightYellowX = 56;
    public static int rightYellowY = -42;
    public static int rightBackup = 5;

    //Experimental coordinates
    public static int purpleX = 13;
    public static int purpleY = -36;

    //Parking coordinates
    public static int parkX = 56;
    public static int parkY = -17;

    //Truss coordinates
    public static int trussTopX = 26;
    public static int trussTopY = -11;
    public static int trussBottomX = -36;
    public static int trussBottomY = -15;

    //White pixels
    public static int whiteStackX = 2;
    public static int whiteStackY = 2;
    public static int whitePixelScoreX = 2;
    public static int whitePixelScoreY = 2;
    @Override
    public void initialize()
    {
        colorProcessor = new PropDetectionProcessor(false);
        super.initialize();

        Pose2d startPose = new Pose2d(15.5, -63.5, Math.toRadians(0));

        roadrunnerMecanumDrive.setPoseEstimate(startPose);

        //Scoring purple pixel
        TrajectorySequence leftPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(150)))
                .waitSeconds(2)
                .build();

        TrajectorySequence middlePurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(purpleX, purpleY, Math.toRadians(90)))
                .waitSeconds(2)
                .build();

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
        TrajectorySequence leftYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPurpleScore.end())
                .lineToLinearHeading(new Pose2d(leftYellowX, leftYellowY, Math.toRadians(0)))
                .waitSeconds(2)
                .build();

        TrajectorySequence middleYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePurpleScore.end())
                .lineToLinearHeading(new Pose2d(centerYellowX, centerYellowY, Math.toRadians(0)))
                .waitSeconds(2)
                .build();

        TrajectorySequence rightYellowScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(rightYellowX, rightYellowY, Math.toRadians(0)))
                .waitSeconds(2)
                .build();

        //right side scoring
        TrajectorySequence rightPurpleScore = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightYellowScore.end())
                .lineToLinearHeading(new Pose2d(rightPurpleX, rightPurpleY, Math.toRadians(0)))
                .turn(Math.toRadians(-190))
                .build();

        //Parking
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

        TrajectorySequence rightPark = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPurpleScore.end())
                .back(6)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .back(24)
                .strafeLeft(10)
                .build();

        //truss
        TrajectorySequence rightTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightPark.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(175)))
                .build();

        TrajectorySequence middleTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(middlePark.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(175)))
                .build();

        TrajectorySequence leftTopTruss = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftPark.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(trussTopX, trussTopY, Math.toRadians(175)))
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

        //white pixel pickup
        TrajectorySequence leftWhitePickup = roadrunnerMecanumDrive.trajectorySequenceBuilder(leftBottomTruss.end())
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(whiteStackX, whiteStackY))
                .waitSeconds(1)
                .build();

        TrajectorySequence middleWhitePickup = roadrunnerMecanumDrive.trajectorySequenceBuilder(middleBottomTruss.end())
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(whiteStackX, whiteStackY))
                .waitSeconds(1)
                .build();

        TrajectorySequence rightWhitePickup = roadrunnerMecanumDrive.trajectorySequenceBuilder(rightBottomTruss.end())
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(whiteStackX, whiteStackY))
                .waitSeconds(1)
                .build();

        //return

        //This just repeats in init
        while(!isStarted())
        {
            /*
             * The camera won't immediately detect a largest contour after being turned on, so it
             * will be (0,0) for a bit, which falls in the LEFT range. After init, wait a few seconds
             * before starting the OpMode so that it detects the right positon.
             */
            propLocation = colorProcessor.GetPropLocation();
            telemetry.addData("Prop Location: ", propLocation);
            telemetry.update();
        }

        mecanumDrive.setBackdropAlignment(false);

        if(isStopRequested()) return;

        //Turn off camera
        if (colorPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
            colorPortal.close();

        if(propLocation.equals("LEFT"))
        {
            schedule(new SequentialCommandGroup(
                    clawL.closeClaw(), clawR.closeClaw(),
                    //arm.transport(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-90))),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPurpleScore)),
                    //arm.pickupFront(),
                    clawL.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftYellowScore)),
                    clawR.openClaw(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftPark)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftTopTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(leftBottomTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
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
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-90))),
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
                    arm.deployFront(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(otherPurple)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-90))),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPurpleScore)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightYellowScore)),
                    clawR.openClaw(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(rightPark)),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightPurpleScore)),
                    arm.pickupFront(),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.turn(Math.toRadians(-190))),
                    clawL.openClaw(),
                    arm.deployFront(),
                    new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightPark)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightTopTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(rightBottomTruss)),
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(leftPostPurple)),
                    //arm.deployFront(),
                    //clawR.openClaw()
                    //new InstantCommand(() -> roadrunnerMecanumDrive.followTrajectory(parkScore)),
                    arm.pickupFront()
            ));
        }
    }

    @Override
    public void run()
    {
        super.run();
        roadrunnerMecanumDrive.update();
        Pose2d poseEstimate = roadrunnerMecanumDrive.getPoseEstimate();

        if(currentSpike)
        {
            telemetry.addLine("Current spike detected! Following broken.");
            roadrunnerMecanumDrive.breakFollowing();
        }

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
