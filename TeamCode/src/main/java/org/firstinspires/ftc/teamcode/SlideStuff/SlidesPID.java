package org.firstinspires.ftc.teamcode.SlideStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //switching to iterative opmode?
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.String;

@Disabled
public class SlidesPID extends LinearOpMode { //switch to iterative opmode?

	/**
	 * DO NOT USE THIS FILE
	 *
	 * IT IS ONLY AN EXAMPLE
	 *
	 * USE THE OTHER SLIDE-RELATED FILES
	 *
	 */
	DcMotorEx motor;
	public SlidesPID(String name) {
		motor = hardwareMap.get(DcMotorEx.class, name);
	}

	// parameters to tune below
	double Kp = 0;
	double Ki = 0;
	double Kd = 0;

	//declaring variables for later use
	ElapsedTime timer = new ElapsedTime();
	double integralSum = 0;
	private double lastError = 0;
	@Override
	public void runOpMode() throws InterruptedException {
		// making variable to represent motor

		// use braking to slow the motor down faster
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// to allow more power when using setPower()
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		waitForStart();

		while (opModeIsActive()) {
			// set motor power to output of PIDControl()
			double power = PIDControl(2000, motor.getCurrentPosition());
			motor.setPower(power);
			if (gamepad1.b) {
				idle();
			}
		}
	}

	public double PIDControl(double target, double state) {
		// PID logic and then return the output
		// obtain the encoder position
		double encoderPosition = motor.getCurrentPosition();

		// calculate the error
		double error = target - encoderPosition;

		// rate of change of the error
		double derivative = (error - lastError) / timer.seconds();

		// sum of all error over time
		integralSum += (error * timer.seconds());

		// saves error to use next time
		lastError = error;

		// resets timer for next calculations
		timer.reset();

		//calculates output and returns
		double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
		return output;
	}
}