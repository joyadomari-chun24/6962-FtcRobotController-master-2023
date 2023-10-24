package org.firstinspires.ftc.teamcode.SlideStuff;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlideBaseSubsystem extends SubsystemBase
{
    /**
     * Based on SlidesPID
     */
    DcMotorEx motor;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    //declaring variables for later use
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    private double lastError = 0;
    public SlideBaseSubsystem(double P, double I, double D, boolean reverseMotor, DcMotorEx motor)
    {
        Kp = P;
        Ki = I;
        Kd = D;
        motor.setDirection(reverseMotor ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public double PIDControl(double target, double state)
    {
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

    private int getEncoderPosition()
    {
        return motor.getCurrentPosition();
    }

    //TODO: integrate extend to target position method here somehow
}
