package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class GyroCheckTest extends OpModeBase
{
    @Override
    public void initialize()
    {
        colorProcessor = new PropDetectionProcessor(true);
        super.initialize();

        Pose2d startPose = new Pose2d(15.5, -63.5, Math.toRadians(0));

        roadrunnerMecanumDrive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = roadrunnerMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13, -39, Math.toRadians(90)))
                .waitSeconds(2)
                .build();

        TrajectorySequence traj2 = roadrunnerMecanumDrive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(33, -30, Math.toRadians(270)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        schedule(new SequentialCommandGroup(
                arm.deployFront(),
                new RunCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(traj1)),
                new RunCommand(() -> gyroCheck(90, 10)),
                new RunCommand(() -> roadrunnerMecanumDrive.followTrajectorySequence(traj2)),
                new RunCommand(() -> gyroCheck(270, 10))
        ));
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
        telemetry.update();
    }
}
