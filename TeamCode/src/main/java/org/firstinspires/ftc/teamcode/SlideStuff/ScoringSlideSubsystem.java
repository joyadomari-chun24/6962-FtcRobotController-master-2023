package org.firstinspires.ftc.teamcode.SlideStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ScoringSlideSubsystem extends SlideBaseSubsystem
{
    public ScoringSlideSubsystem(DcMotorEx scoringMotor)
    {
        super(0, 0, 0, true, scoringMotor);
        scoringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoringMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void periodic()
    {
        /*double power = PIDControl(700, motor.getCurrentPosition());
        motor.setPower(power);*/
    }
}