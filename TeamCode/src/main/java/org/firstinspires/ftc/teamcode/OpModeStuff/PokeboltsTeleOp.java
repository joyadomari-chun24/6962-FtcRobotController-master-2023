package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;

import org.firstinspires.ftc.teamcode.SlideStuff.ScoringSlideSubsystem;


@TeleOp(name="PokéboltsTeleOp", group="Active TeleOps")
public class PokeboltsTeleOp extends OpModeBase
{

    @Override
    public void initialize()
    {
        super.initialize();

        //aprilPortal.stopStreaming();
        mecanumDrive.setBackdropAlignment(false);
        clawL.autoClosing = true;
        clawR.autoClosing = true;

        /*
        * Gamepad 2:
        *
        * Left joystick - Scoring Slides
        *
        * Left joystick button - Back score
        *
        * D-Pad left and right - Adjustable Wrist
        *
        * D-Pad Up and Down - Adjustable Arm
        *
        * Right bumper - Right claw Toggle
        *
        * Left bumper - left claw toggle
        *
        * A - Ground Pickup Position
        *
        * B - Back score position (without slides)
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
        * Left joystick button - straight drive
        *
        * Right joystick - turn
        *
        * Left Bumper - slow mode
        *
        * Right Bumper - align to backdrop
        *
        * Left D-Pad - strafe left
        *
        * Right D-Pad - strafe right
        *
        * B - reset gyro
        *
        * X and Y - Drone launch
        *
        * A and X - Hang
        *
        * */

        //Slow mode
        gamepadEx1.getGamepadButton(LEFT_BUMPER).whileHeld(mecanumDrive.slowFieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroManager::getHeading, telemetry));

        //Straight drives
        gamepadEx1.getGamepadButton(LEFT_STICK_BUTTON).whileHeld(mecanumDrive.roboCentric(0.0, 0.5, 0.0));
        gamepadEx1.getGamepadButton(DPAD_LEFT).whileHeld(mecanumDrive.roboCentric(-0.5, 0.0, 0.0));
        gamepadEx1.getGamepadButton(DPAD_RIGHT).whileHeld(mecanumDrive.roboCentric(0.5, 0.0, 0.0));

        //Align with backdrop
        gamepadEx1.getGamepadButton(RIGHT_BUMPER).whileHeld(new InstantCommand(() -> mecanumDrive.alignToBackdrop()));

        //Drone launcher
        gamepadEx1.getGamepadButton(X).and(gamepadEx1.getGamepadButton(Y)).toggleWhenActive(launcher.setDrone(), launcher.fireDrone());

        //Hang
        gamepadEx1.getGamepadButton(X).and(gamepadEx1.getGamepadButton(A)).whileActiveOnce(hang.hangRobot());
        
        //Claw
        gamepadEx2.getGamepadButton(RIGHT_BUMPER).toggleWhenPressed(clawR.openClaw(), clawR.closeClaw());
        gamepadEx2.getGamepadButton(LEFT_BUMPER).toggleWhenPressed(clawL.openClaw(), clawL.closeClaw());
        //tyler controller set to close, isaiah controller set to open, request from drivers. Tyler said its more effecient because he can intake better, while isaiah can focus on scoring

        //Arm/wrist/slide positions (whenActive and whileHeld both work, so they're split among the buttons here for fun)
        gamepadEx2.getGamepadButton(A).whenActive(arm.pickupFront());
        gamepadEx2.getGamepadButton(B).whileHeld(arm.deployBack());
        gamepadEx2.getGamepadButton(Y).whenActive(arm.deployFront());
        gamepadEx2.getGamepadButton(X).whileHeld(arm.transport());
        gamepadEx2.getGamepadButton(LEFT_STICK_BUTTON).toggleWhenPressed(new SequentialCommandGroup(arm.transport(), scoringSlides.extendToPosition(800), arm.deployBack()), new SequentialCommandGroup(arm.transport(), scoringSlides.extendToPosition(0), arm.deployFront()));

        //Adjustable arm
        gamepadEx2.getGamepadButton(DPAD_UP).whileHeld(arm.incrementalArm(-1));
        gamepadEx2.getGamepadButton(DPAD_DOWN).whileHeld(arm.incrementalArm(1));

        //Adjustable wrist
        gamepadEx2.getGamepadButton(DPAD_LEFT).whileHeld(arm.incrementalWrist(1));
        gamepadEx2.getGamepadButton(DPAD_RIGHT).whileHeld(arm.incrementalWrist(-1));

        //Gyro reset
        gamepadEx1.getGamepadButton(B).whenActive(gyroManager::reset);

        //Slides
        scoringSlides.setDefaultCommand(scoringSlides.slideMovement(gamepadEx2::getRightY));

        //Temporary apriltag experiment
        gamepadEx1.getGamepadButton(DPAD_UP).whileHeld(new InstantCommand(() -> driveToAprilTag(redTagId)));

        mecanumDrive.setDefaultCommand(mecanumDrive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroManager::getHeading, telemetry));
        telemetry.log().clear();
        telemetry.log().add("TeleOpTest2 has initialized.");
        telemetry.update();

        waitForStart();

        //Turn off cameras to save bandwidth
//        if (aprilPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
//            aprilPortal.close();
//        if (colorPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
//            colorPortal.close();

    }

    @Override
    public void run()
    {
        super.run();

        //Update the roadrunner chassis code so that it can create the pose estimate (but we're not using it to drive)
        roadrunnerMecanumDrive.update();
        Pose2d poseEstimate = roadrunnerMecanumDrive.getPoseEstimate();

        //Telemetry
        telemetry.addData("L Slide Position ", scoringSlideMotorL.getCurrentPosition());
        telemetry.addData("R Slide Position ", scoringSlideMotorR.getCurrentPosition());
        telemetry.addData("Slide Controlling Joystick", gamepadEx2.getRightY());
        telemetry.addData("Left Arm Position", leftPlatformServo.getPosition());
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.addData("FR Current (Amps)", rightFrontCurrentReader.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Gyro Heading ", gyroManager.getHeading());
        telemetry.addData("x cord", poseEstimate.getX());
        telemetry.addData("y cord", poseEstimate.getY());
        telemetry.addData("roadrunner predicted heading", poseEstimate.getHeading());
        telemetry.addData("Claw Target Position Left ", clawServoL.getPosition());
        telemetry.addData("Claw Target Position Right ", clawServoR.getPosition());
        telemetry.addData("isOpen Left", clawL.getOpen());
        telemetry.addData("isOpen Right", clawR.getOpen());
        telemetry.addData("Color Sensor Right ", colorSensorR.getDistance(DistanceUnit.INCH));
        telemetry.addData("Color Sensor Left ", colorSensorL.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Arm Position", rightPlatformServo.getPosition());
        telemetry.addData("Hang Position Left ", hangServoL.getPosition());
        telemetry.addData("Hang Position Right ", hangServoR.getPosition());
        telemetry.addData("Drone servo position", droneServo.getPosition());
        telemetry.update();

    }

}
