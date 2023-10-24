package org.firstinspires.ftc.teamcode.SlideStuff;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ScoringSlideSubsystem extends SlideBaseSubsystem
{
    public ScoringSlideSubsystem(DcMotorEx scoringMotor)
    {
        super(0, 0, 0, true, scoringMotor);
    }
    @Override
    public void periodic()
    {
        //WIP
        double power = PIDControl(700, motor.getCurrentPosition());
        motor.setPower(power);
    }
}