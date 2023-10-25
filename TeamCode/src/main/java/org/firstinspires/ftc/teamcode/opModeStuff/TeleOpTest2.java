package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MecanumDriveSubsystem;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;


@TeleOp
public class TeleOpTest2 extends OpModeBase
{
    private double gyroAngle;

    @Override
    public void initialize()
    {
        super.initialize();

        //Todo: figure out the gyro thing for field oriented
        //I think that the problem is that it's only calling the angle once and using that as the robot's heading.


        //(add the listeners for button inputs that lead to commands here)

        //This slow mode code definitely doesn't work
        if (gamepadEx1.getButton(LEFT_BUMPER))
            mecanumDrive.slowFieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroAngle);

        //even though it's being set, it doesn't drive field oriented for some reason
        mecanumDrive.setDefaultCommand(mecanumDrive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroAngle));
        telemetry.log().add("TeleOpTest2 has initialized.");
        telemetry.update();
    }

    public void run()
    {
        super.run();

        gyroAngle = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Update the roadrunner chassis code so that it can create the pose estimate (but we're not using it to drive)
        roadrunnerMecanumDrive.update();
        Pose2d poseEstimate = roadrunnerMecanumDrive.getPoseEstimate();

        //Telemetry :)
        telemetry.addData("LeftStickX", gamepadEx1.getLeftX());
        telemetry.addData("LeftStickY", gamepadEx1.getLeftY());
        telemetry.addData("RightStickX", gamepadEx1.getRightX());
        telemetry.addData("Gyro Heading", gyroAngle);
        telemetry.addData("x cord", poseEstimate.getX());
        telemetry.addData("y cord", poseEstimate.getY());
        telemetry.addData("roadrunner predicted heading", poseEstimate.getHeading());
        telemetry.update();

    }
}
