package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SlideStuff.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.SlideStuff.SlideBaseSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
public class TeleOpTest extends OpModeBase
{
    @Override
    public void initialize()
    {
        super.initialize();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeSlideSubsystem test = new IntakeSlideSubsystem();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("TeleOpTest has initialized.");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    //(put methods here)
}
