package org.firstinspires.ftc.teamcode.SlideStuff;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class ScoringSlideSubsystem extends SlideBaseSubsystem
{
    private DcMotorEx motorLeft, motorRight;
    private double slidePower;
    public int target = 0;
    public ScoringSlideSubsystem(DcMotorEx scoringMotorL, DcMotorEx scoringMotorR)
    {
        super(0, 0, 0, true, false, scoringMotorL, scoringMotorR);
        motorLeft = scoringMotorL;
        motorRight = scoringMotorR;
        //The slides still just fall down btw (need PID to hold)
    }
    @Override
    public void periodic()
    {
        double powerL = PIDControl(target, motorLeft.getCurrentPosition());
        double powerR = PIDControl(target, motorRight.getCurrentPosition());
        if (slidePower != 0)
        {
            motorLeft.setPower(slidePower);
            motorRight.setPower(slidePower);
            target = motorLeft.getCurrentPosition();
        }
        else
        {
            motorLeft.setPower(powerL);
            motorRight.setPower(powerR);
        }
        super.periodic();
    }

    public Command slideMovement(DoubleSupplier motorPower)
    {
        return new RunCommand(() -> {slidePower = motorPower.getAsDouble();}, this);
    }
}