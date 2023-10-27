package org.firstinspires.ftc.teamcode.SlideStuff;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class ScoringSlideSubsystem extends SlideBaseSubsystem
{
    private DcMotorEx motorForScoring;
    public ScoringSlideSubsystem(DcMotorEx scoringMotor)
    {
        super(0, 0, 0, true, scoringMotor);
        motorForScoring = scoringMotor;
    }
    @Override
    public void periodic()
    {
//        double power = PIDControl(700, motor.getCurrentPosition());
//        motor.setPower(power);
    }

    public Command slideMovement(DoubleSupplier motorPower)
    {
        return new RunCommand(() -> {motorForScoring.setPower(motorPower.getAsDouble());});
    }
}