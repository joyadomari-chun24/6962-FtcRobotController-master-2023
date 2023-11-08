package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import org.firstinspires.ftc.teamcode.SlideStuff.ScoringSlideSubsystem;


@TeleOp(name="Use This TeleOp")
public class TeleOpTest2 extends OpModeBase
{
    private double gyroAngle;
    private double armIncrement = 0.025;
    private double wristIncrement = 0.01;


    @Override
    public void initialize()
    {

        super.initialize();
        arm.pickupFront();


        /*
        * Gamepad 2:
        *
        * Right joystick - Scoring Slides
        *
        * Left joystick - Adjustable Wrist (Currently D-Pad Left and Right)
        *
        * D-Pad Up and Down - Adjustable Arm
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
        * */

        /*
        * Gamepad 1:
        *
        * Left joystick - strafe
        *
        * Right joystick - turn
        *
        * Left Bumper - slow mode
        *
        * X and Y - Drone launch
        *
        * */

        //Slow mode
        gamepadEx1.getGamepadButton(LEFT_BUMPER).whileHeld(mecanumDrive.slowFieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroManager::getHeading, telemetry));

        //Drone launcher
        gamepadEx1.getGamepadButton(X).and(gamepadEx1.getGamepadButton(Y)).whileActiveOnce(launcher.fireDrone());

        //Claw (still needs 3 positions)
        //non-toggleable claw
        gamepadEx2.getGamepadButton(LEFT_BUMPER).whileHeld(claw.openClaw());
        gamepadEx2.getGamepadButton(RIGHT_BUMPER).whileHeld(claw.closeClaw());
        //toggleable claw
        //gamepadEx2.getGamepadButton(RIGHT_BUMPER).toggleWhenPressed(new InstantCommand(claw::openClaw), new InstantCommand(claw::closeClaw));

        //Arm/wrist positions
        //whenActive and whileHeld seem like they should both work, so they're split rn to test
        gamepadEx2.getGamepadButton(A).whenActive(arm.pickupFront());
        gamepadEx2.getGamepadButton(Y).whenActive(arm.deployFront());
        gamepadEx2.getGamepadButton(X).whileHeld(arm.topDown());
        gamepadEx2.getGamepadButton(B).whileHeld(arm.deployBack());

        //Adjustable arm

        gamepadEx2.getGamepadButton(DPAD_UP).whileHeld(arm.incrementalArm(armIncrement));
        gamepadEx2.getGamepadButton(DPAD_DOWN).whileHeld(arm.incrementalArm(-1*armIncrement));

        //Adjustable wrist
        gamepadEx2.getGamepadButton(DPAD_LEFT).whileHeld(arm.incrementalWrist(wristIncrement));
        gamepadEx2.getGamepadButton(DPAD_RIGHT).whileHeld(arm.incrementalWrist(-1*wristIncrement));

        //Slides (set position needs work)
        gamepadEx2.getGamepadButton(LEFT_STICK_BUTTON).toggleWhenPressed(scoringSlides.extendToPosition(500), scoringSlides.extendToPosition(0));
        scoringSlides.setDefaultCommand(scoringSlides.slideMovement(gamepadEx2::getRightY));

        mecanumDrive.setDefaultCommand(mecanumDrive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroManager::getHeading, telemetry));
        telemetry.log().clear();
        telemetry.log().add("TeleOpTest2 has initialized.");
        telemetry.update();
    }

    @Override
    public void run()
    {
        super.run();

        //Update the roadrunner chassis code so that it can create the pose estimate (but we're not using it to drive)
        roadrunnerMecanumDrive.update();
        Pose2d poseEstimate = roadrunnerMecanumDrive.getPoseEstimate();

        //Telemetry
        telemetry.addData("LeftStickX (pad1)", gamepadEx1.getLeftX());
        telemetry.addData("LeftStickY (pad1)", gamepadEx1.getLeftY());
        telemetry.addData("RightStickX (pad1)", gamepadEx1.getRightX());
        telemetry.addData("L Slide Position (method)", scoringSlides.getPosition(ScoringSlideSubsystem.motorSide.LEFT));
        telemetry.addData("R Slide Position (method)", scoringSlides.getPosition(ScoringSlideSubsystem.motorSide.RIGHT));
        telemetry.addData("L Slide Position (straight)", scoringSlideMotorL.getCurrentPosition());
        telemetry.addData("R Slide Position (straight)", scoringSlideMotorR.getCurrentPosition());
        telemetry.addData("Gyro Heading ", gyroManager.getHeading());
        telemetry.addData("x cord", poseEstimate.getX());
        telemetry.addData("y cord", poseEstimate.getY());
        telemetry.addData("roadrunner predicted heading", poseEstimate.getHeading());
        telemetry.addData("Claw Target Position", clawServo.getPosition());
        telemetry.addData("Left Arm Position", leftPlatformServo.getPosition());
        telemetry.addData("Right Arm Position", rightPlatformServo.getPosition());
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.update();

    }
}
