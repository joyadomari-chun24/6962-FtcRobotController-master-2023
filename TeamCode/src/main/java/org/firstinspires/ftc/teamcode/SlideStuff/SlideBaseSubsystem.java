package org.firstinspires.ftc.teamcode.SlideStuff;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlideBaseSubsystem extends SubsystemBase
{
    /**
     * Based on SlidesPID
     */
    DcMotorEx motor1, motor2;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    public static double Kg = 0;//tune till the slide holds itself in place


    //declaring variables for later use
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    private double lastError = 0;

    //Constructor for single motor
    public SlideBaseSubsystem(double P, double I, double D, boolean reverseMotor, DcMotorEx motor)
    {
        motor1 = motor;
        Kp = P;
        Ki = I;
        Kd = D;
        motor1.setDirection(reverseMotor ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Alternate constructor for double motors
    public SlideBaseSubsystem(double P, double I, double D, boolean reverseMotorL, boolean reverseMotorR, DcMotorEx leftMotor, DcMotorEx rightMotor)
    {
        motor1 = leftMotor;
        motor2 = rightMotor;
        Kp = P;
        Ki = I;
        Kd = D;
        motor1.setDirection(reverseMotorL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(reverseMotorR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double PIDControl(double target, double state, DcMotorEx motor)
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
        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + Kg;
        return output;
    }

    //Not sure if this works, so I commented it out for now to avoid confusion
//    private int getEncoderPosition(DcMotorEx motor)
//    {
//        return motor.getCurrentPosition();
//    }

    //Commented out for testing in more specific subsystems
//    public Command extendToPosition(int targetPos, int currentState, DcMotorEx motor)
//    {
//        return new RunCommand(() -> {while(Math.abs(targetPos-currentState) > 50) {PIDControl(targetPos, currentState, motor);}});
//    }
}
