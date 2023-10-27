package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MecanumDriveSubsystem;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;


@TeleOp
public class TeleOpTest2 extends OpModeBase
{
    private double gyroAngle;

    @Override
    public void initialize()
    {
        super.initialize();

        /*
        * Gamepad 2:
        *
        * Left joystick - Scoring Slides
        *
        * D-Pad - Adjustable Arm
        *
        * Right bumper - Claw
        *
        * A - Ground Pickup Position
        *
        * B - Back Scoring Position
        *
        * Y - Front Scoring Position
        *
        * X - Top Down Position
        *
        * Right joystick - Adjustable Wrist
        *
        * */

        /*
        * Gamepad 1:
        *
        * Left joystick - strafe
        *
        * Right joystick - turn
        *
        * */

        //Todo: figure out the gyro thing for field oriented
        //I think that the problem is that it's only calling the angle once and using that as the robot's heading.
        //ToggleButtonReader clawToggle = new ToggleButtonReader(gamepadEx2, RIGHT_BUMPER);

        //Slow mode
        gamepadEx1.getGamepadButton(LEFT_BUMPER).whileHeld(mecanumDrive.slowFieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroAngle, telemetry));

        //Claw
        gamepadEx1.getGamepadButton(A).whileHeld(claw.openClaw());
        gamepadEx1.getGamepadButton(B).whileHeld(claw.closeClaw());

        //Debugging arm/wrist
        gamepadEx1.getGamepadButton(Y).whileHeld(arm.position1());
        gamepadEx1.getGamepadButton(X).whileHeld(arm.positionW());

        //Arm/wrist positions
        gamepadEx2.getGamepadButton(A).whileHeld(arm.pickupFront());
        gamepadEx2.getGamepadButton(B).whileHeld(arm.deployBack());
        gamepadEx2.getGamepadButton(Y).whileHeld(arm.deployFront());
        gamepadEx2.getGamepadButton(X).whileHeld(arm.topDown());

        //Adjustable arm (not sure if this'll work)
        gamepadEx2.getGamepadButton(DPAD_UP).whileHeld(arm.incrementalArm(5));
        gamepadEx2.getGamepadButton(DPAD_DOWN).whileHeld(arm.incrementalArm(-5));


        //To do: get slides to work
        //Not sure why this keeps on sending the error "default command requires the subsystem!" ???
        //outtakeSlides.setDefaultCommand(outtakeSlides.slideMovement(gamepadEx2::getLeftY));

        //even though it's being set, it doesn't drive field oriented for some reason
        mecanumDrive.setDefaultCommand(mecanumDrive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroAngle, telemetry));
        telemetry.log().clear();
        telemetry.log().add("TeleOpTest2 has initialized.");
        telemetry.update();
    }

    @Override
    public void run()
    {
        super.run();

        gyroAngle = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Update the roadrunner chassis code so that it can create the pose estimate (but we're not using it to drive)
        roadrunnerMecanumDrive.update();
        Pose2d poseEstimate = roadrunnerMecanumDrive.getPoseEstimate();

        //Temp until I figure out how to use the PID loop
        scoringSlideMotor.setPower(gamepadEx2.getLeftY());

        //Telemetry
        telemetry.addData("LeftStickX", gamepadEx1.getLeftX());
        telemetry.addData("LeftStickY", gamepadEx1.getLeftY());
        telemetry.addData("RightStickX", gamepadEx1.getRightX());
        telemetry.addData("Gyro Heading", gyroAngle);
        telemetry.addData("x cord", poseEstimate.getX());
        telemetry.addData("y cord", poseEstimate.getY());
        telemetry.addData("roadrunner predicted heading", poseEstimate.getHeading());
        telemetry.addData("Claw Position", clawServo.getPosition());
        telemetry.addData("Left Arm Position", leftPlatformServo.getPosition());
        telemetry.addData("Right Arm Position", rightPlatformServo.getPosition());
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.addData("Slow mode pressed? ", gamepadEx1.getGamepadButton(LEFT_BUMPER));
        telemetry.update();

    }
}
