package org.firstinspires.ftc.teamcode.SlideStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class ScoringSlideSubsystem extends SubsystemBase
{
    /**
     * Based on SlidesPID
     */
    private DcMotorEx motorLeft, motorRight;
    private double slidePower;
    private int joystickScalar = 1;
    private int target = 0;
    public enum motorSide {LEFT, RIGHT}
    public Telemetry telemetry;
    FtcDashboard dashboard = FtcDashboard.getInstance();;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kg = 0; //tune till the slide holds itself in place

    //declaring variables for later use
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    public double lastError = 0;
    public ScoringSlideSubsystem(DcMotorEx scoringMotorL, DcMotorEx scoringMotorR, boolean reverseMotorL, boolean reverseMotorR, Telemetry tel)
    {
        motorLeft = scoringMotorL;
        motorRight = scoringMotorR;
        telemetry = tel;

//        Kp = P;
//        Ki = I;
//        Kd = D;
        motorLeft.setDirection(reverseMotorL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(reverseMotorR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void periodic()
    {
        super.periodic();

        double powerL = PIDControl(target, motorLeft);
        double powerR = PIDControl(target, motorRight);
        // First part sets motors to the joystick if the joystick is moving and sets target to it so slides stay when let go
        if (Math.abs(slidePower) > 0.05)
        {
            motorLeft.setPower(slidePower/2);
            motorRight.setPower(slidePower/2);
            target = motorLeft.getCurrentPosition() + (int) slidePower * joystickScalar;
        }
        else //If not, move slides to target (current pos from joystick+1 or a button's set position)
        {
            motorLeft.setPower(powerL);
            motorRight.setPower(powerR);
        }
        telemetry.addData("PID Power L", powerL);
        telemetry.addData("PID Power R", powerR);
    }

    public Command extendToPosition(int targetPos)
    {
        return new InstantCommand(() -> target = targetPos);
    }

    public Command slideMovement(DoubleSupplier motorPower)
    {
        return new RunCommand(() -> {slidePower = motorPower.getAsDouble();}, this);
    }

    public int getPosition(motorSide motorSide)
    {
        return (motorSide == motorSide.LEFT) ? motorLeft.getCurrentPosition() : motorRight.getCurrentPosition();
    }

    public double PIDControl(double target, DcMotorEx motor)
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
}