package org.firstinspires.ftc.teamcode.SlideStuff;

public class ScoringSlideSubsystem extends SlideBaseSubsystem
{
    public ScoringSlideSubsystem()
    {
        super(0, 0, 0, true);
    }
    @Override
    public void periodic()
    {
        double power = PIDControl(100, motor.getCurrentPosition());
        motor.setPower(power);
    }
}