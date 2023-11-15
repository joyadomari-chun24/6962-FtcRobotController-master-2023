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
    public static double sustainedPosition = 0;
    public static double releasedPosition = 1;
    public HangSubsystem(Servo hangServoL, Servo hangServoR)
    {
        lever = hangServoL;
        lever2 = hangServoR;
        lever.setPosition(sustainedPosition);
        lever.setPosition(sustainedPosition);
    }
    public Command hangRobot()
    {
        return new InstantCommand(() -> {lever.setPosition(releasedPosition); lever2.setPosition(releasedPosition);});
    }
}
