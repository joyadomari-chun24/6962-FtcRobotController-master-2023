package org.firstinspires.ftc.teamcode.EndgameStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HangSubsystem extends SubsystemBase
{
    private Servo lever, lever2;
    public static double sustainedPosition = 0.7;
    public static double releasedPosition = 0;
    public HangSubsystem(Servo hangServoL, Servo hangServoR)
    {
        lever = hangServoL;
        lever2 = hangServoR;
        lever.setPosition(sustainedPosition);
        lever2.setPosition(1-sustainedPosition);
    }
    public Command hangRobot()
    {
        return new InstantCommand(() -> {lever.setPosition(releasedPosition); lever2.setPosition(1-releasedPosition);});
    }
}
