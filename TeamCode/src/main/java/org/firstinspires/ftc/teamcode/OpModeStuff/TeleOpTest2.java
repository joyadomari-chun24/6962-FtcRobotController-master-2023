package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import org.firstinspires.ftc.teamcode.SlideStuff.ScoringSlideSubsystem;


@TeleOp(name="Use This TeleOp")
public class TeleOpTest2 extends OpModeBase
{

    private double gyroAngle;
    private double armIncrement = 0.020;
    private double wristIncrement = 0.01;


    @Override
    public void initialize()
    {

        super.initialize();

        /*
        * Gamepad 2:
        *
        * Left joystick - Scoring Slides
        *
        * Right joystick - Adjustable Wrist (Currently D-Pad Left and Right)
        *
        * D-Pad Up and Down - Adjustable Arm
        *
        * Right bumper - Claw Toggle
        *
        * A - Ground Pickup Position
        *
        * B - Retracted Position
        *
        * Y - Front Scoring Position
        *
        * X - Transport Position
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
        * B - reset gyro
        *
        * D-Pad Up - drive to backdrop (not implemented)
        *
        * D-Pad Down - drive to human player station (not implemented)
        *
        * X and Y - Drone launch
        *
        * */

        //Slow mode
        gamepadEx1.getGamepadButton(LEFT_BUMPER).whileHeld(mecanumDrive.slowFieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroManager::getHeading, telemetry));

        //Drone launcher gamepad 1  x+y
        gamepadEx1.getGamepadButton(X).and(gamepadEx1.getGamepadButton(Y)).whileActiveOnce(launcher.fireDrone());
        //hang gamepad 1  a+b
        gamepadEx1.getGamepadButton(B).and(gamepadEx1.getGamepadButton(A)).whileActiveOnce(hang.hangRobot());

        //Claw
        //tyler controller set to close, isaiah controller set to open, request from drivers. Tyler said its more effecient because he can intake better, while isaiah can focus on scoring
//        gamepadEx2.getGamepadButton(RIGHT_BUMPER).toggleWhenPressed(claw.openClaw(), claw.closeClaw());
        gamepadEx1.getGamepadButton(RIGHT_BUMPER).toggleWhenPressed(claw.closeClaw());
        gamepadEx2.getGamepadButton(RIGHT_BUMPER).toggleWhenPressed(claw.openClaw());

        //Arm/wrist positions
        //whenActive and whileHeld seem like they should both work, so they're split rn to test
        gamepadEx2.getGamepadButton(A).whenActive(arm.pickupFront());
        gamepadEx2.getGamepadButton(B).whileHeld(arm.deployBack());
        gamepadEx2.getGamepadButton(Y).whenActive(arm.deployFront());
        gamepadEx2.getGamepadButton(X).whileHeld(arm.transport());

        //Adjustable arm
        gamepadEx2.getGamepadButton(DPAD_UP).whileHeld(arm.incrementalArm(1));
        gamepadEx2.getGamepadButton(DPAD_DOWN).whileHeld(arm.incrementalArm(-1));

        //Adjustable wrist
        gamepadEx2.getGamepadButton(DPAD_LEFT).whileHeld(arm.incrementalWrist(1));
        gamepadEx2.getGamepadButton(DPAD_RIGHT).whileHeld(arm.incrementalWrist(-1));
        //incremental bound to joystick returns error that we're not sure how to fix yet
        //arm.setDefaultCommand(arm.incrementalWrist(gamepadEx2::getRightY));

        //Gyro reset
        gamepadEx1.getGamepadButton(B).whenActive(gyroManager::reset);

        //Slides
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
        telemetry.addData("Hang Position", hangServo.getPosition());
        telemetry.addData("Drone servo position", droneServo.getPosition());
        telemetry.update();

    }
}
