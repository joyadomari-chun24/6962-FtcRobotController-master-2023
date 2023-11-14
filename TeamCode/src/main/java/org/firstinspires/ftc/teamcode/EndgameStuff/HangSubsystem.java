package org.firstinspires.ftc.teamcode.EndgameStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HangSubsystem extends SubsystemBase
{
    private Servo lever;
    public static double sustainedPosition = 0;
    public static double releasedPosition = 1;
    public HangSubsystem(Servo hangServo)
    {
        lever = hangServo;
        lever.setPosition(sustainedPosition);
    }
    public Command hangRobot()
    {
        return new InstantCommand(() -> {lever.setPosition(releasedPosition);});
    }
}
