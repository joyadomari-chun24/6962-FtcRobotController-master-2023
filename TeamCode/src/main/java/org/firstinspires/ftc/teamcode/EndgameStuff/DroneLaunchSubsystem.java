package org.firstinspires.ftc.teamcode.EndgameStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneLaunchSubsystem extends SubsystemBase
{
    private Servo trigger;
    public static double primedPosition = 0.7;
    public static double openPosition = 0.38;
    public DroneLaunchSubsystem(Servo droneLauncher)
    {
        trigger = droneLauncher;
        trigger.setPosition(primedPosition);
    }
    public Command fireDrone()
    {
        return new InstantCommand(() -> {trigger.setPosition(openPosition);});
    }

    public Command setDrone()
    {
        return new InstantCommand(() -> {trigger.setPosition(primedPosition);});
    }
}
