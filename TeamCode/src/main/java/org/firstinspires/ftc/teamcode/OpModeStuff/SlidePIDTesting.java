package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlidePIDTesting extends OpModeBase
{
	public static int extension = 1000;
	@Override
	public void initialize()
	{
		super.initialize();
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
			telemetry.addData("Left position (straight): ", scoringSlideMotorL.getCurrentPosition());
			telemetry.addData("Right position (straight): ", scoringSlideMotorR.getCurrentPosition());
			telemetry.update();
		}
	}
}