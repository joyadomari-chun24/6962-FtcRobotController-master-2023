package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
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
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;


@TeleOp(name="Use This TeleOp")
public class TeleOpTest2 extends OpModeBase
{
    private double gyroAngle;
    private double armIncrement = 0.005;
    private double wristIncrement = 0.005;

    @Override
    public void initialize()
    {
        super.initialize();

        /*
        * Gamepad 2:
        *
        * Left joystick - Scoring Slides (currently bound to triggers)
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
        * Right joystick - Adjustable Wrist (not implemented yet)
        *
        * */

        /*
        * Gamepad 1:
        *
        * Left joystick - strafe
        *
        * Right joystick - turn
        *
        * X - Drone launch
        *
        * */

        //Todo: figure out the gyro thing for field oriented
        //I think that the problem is that it's only calling the angle once and using that as the robot's heading.

        //Slow mode
        gamepadEx1.getGamepadButton(LEFT_BUMPER).whileHeld(mecanumDrive.slowFieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, gyroAngle, telemetry));

        //Drone launcher
        gamepadEx1.getGamepadButton(X).and(gamepadEx1.getGamepadButton(Y)).whileActiveOnce(launcher.fireDrone());

        //Claw
        gamepadEx1.getGamepadButton(LEFT_BUMPER).whileHeld(claw.openClaw());
        gamepadEx1.getGamepadButton(RIGHT_BUMPER).whileHeld(claw.closeClaw());
        gamepadEx2.getGamepadButton(RIGHT_BUMPER).whenPressed(new ConditionalCommand(new InstantCommand(claw::openClaw), new InstantCommand(claw::closeClaw), () -> {return claw.toggle();}));

        //Arm/wrist positions
        gamepadEx2.getGamepadButton(A).whileHeld(arm.pickupFront());
        gamepadEx2.getGamepadButton(Y).whileHeld(arm.deployFront());
        gamepadEx2.getGamepadButton(X).whileHeld(arm.topDown());
        // no need yet
        //gamepadEx2.getGamepadButton(B).whileHeld(arm.deployBack());

        //Adjustable arm (NEEDS WORK)
        gamepadEx2.getGamepadButton(DPAD_UP).whileHeld(arm.incrementalArm(armIncrement));
        gamepadEx2.getGamepadButton(DPAD_DOWN).whileHeld(arm.incrementalArm(-1*armIncrement));

        //Adjustable wrist
        gamepadEx2.getGamepadButton(DPAD_LEFT).whileHeld(arm.incrementalWrist(wristIncrement));
        gamepadEx2.getGamepadButton(DPAD_RIGHT).whileHeld(arm.incrementalWrist(-1*wristIncrement));

        //Slides
        //???

        //To do: get slides to work
        //Not sure why this keeps on sending the error "default command requires the subsystem!" ???
        //outtakeSlides.setDefaultCommand(outtakeSlides.slideMovement(gamepadEx2::getLeftY));

        //even though it's being set, it doesn't drive field oriented for some reason
        mecanumDrive.setDefaultCommand(mecanumDrive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX,gyroAngle, telemetry));
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

        //Temp until I figure out how to use the PID loop (apparently this doesn't work)
        scoringSlideMotor.setPower(gamepad2.left_stick_y);

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
