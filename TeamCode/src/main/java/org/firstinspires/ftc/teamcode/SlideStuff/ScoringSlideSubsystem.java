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
    private double slideScalar = 0.45;
    private int target = 0;
    public enum motorSide {LEFT, RIGHT}
    public Telemetry telemetry;
    FtcDashboard dashboard = FtcDashboard.getInstance();;
    public static double Kp = 0.006;
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
            // if position positive, then can move
            if (motorLeft.getCurrentPosition() > -2) {
                motorLeft.setPower(slidePower * slideScalar);
                motorRight.setPower(slidePower * slideScalar);
                target = motorLeft.getCurrentPosition();
            }
            // if position negative, but slidePower is positive, then can move
            else if (motorLeft.getCurrentPosition() <= -2 && slidePower > 0)
            {
                motorLeft.setPower(slidePower * slideScalar);
                motorRight.setPower(slidePower * slideScalar);
                target = motorLeft.getCurrentPosition();
            }
            // could add "else {target = 0;}", but seems unnecessary, and could cause problems
        }
        else //If not, move slides to target (current pos from joystick+1 or a button's set position)
        {
            motorLeft.setPower(powerL);
            motorRight.setPower(powerR);
        }
        telemetry.addData("PID Power L", powerL);
        telemetry.addData("PID Power R", powerR);
        telemetry.addData("Slide Target ", target);
    }

    public Command extendToPosition(int targetPos)
    {
        return new InstantCommand(() -> target = targetPos);
    }

    public Command slideMovement(DoubleSupplier motorPower)
    {
        return new RunCommand(() -> {slidePower = -1 * motorPower.getAsDouble();}, this);
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