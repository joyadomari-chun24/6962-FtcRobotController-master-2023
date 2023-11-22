package org.firstinspires.ftc.teamcode.SlideStuff;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStuff.OpModeBase;

@TeleOp
public class SlidePIDTesting extends OpModeBase
{
	public static int extension = 1000;
	@Override
	public void initialize()
	{
		super.initialize();
		gamepadEx2.getGamepadButton(A).toggleWhenPressed(scoringSlides.extendToPosition(2000), scoringSlides.extendToPosition(0));
		scoringSlides.setDefaultCommand(scoringSlides.slideMovement(gamepadEx2::getRightY));
	}

	@Override
	public void run()
	{
		if (opModeIsActive())
		{
			// set motor power to output of PIDControl()
			double powerL = scoringSlides.PIDControl(extension, scoringSlideMotorL);
			double powerR = scoringSlides.PIDControl(extension, scoringSlideMotorR);
			scoringSlideMotorL.setPower(powerL);
			scoringSlideMotorR.setPower(powerR);

			if (gamepad1.b)
			{
				idle();
			}
			telemetry.addData("powerL: ", powerL);
			telemetry.addData("powerR: ", powerR);
			telemetry.addData("Left position: ", scoringSlideMotorL.getCurrentPosition());
			telemetry.addData("Right position: ", scoringSlideMotorR.getCurrentPosition());
			telemetry.addData("measuredVelocityL: ", scoringSlideMotorL.getVelocity());
			telemetry.addData("measuredVelocityR: ", scoringSlideMotorR.getVelocity());
			telemetry.update();
		}
	}
}