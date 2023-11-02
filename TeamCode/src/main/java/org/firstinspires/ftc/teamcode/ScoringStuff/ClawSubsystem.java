package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawSubsystem extends SubsystemBase
{
    private Servo claw;
    private double closedPosition = 0;
    private double openPosition = 1;
    private boolean clawToggle = false;
    public ClawSubsystem(Servo theClaw)
    {
        claw = theClaw;
    }

    public boolean toggle()
    {
        clawToggle = !clawToggle;
        return clawToggle;
    }
    public Command closeClaw()
    {
        return new InstantCommand(() -> {claw.setPosition(closedPosition);});
    }

    public Command openClaw()
    {
        return new InstantCommand(() -> {claw.setPosition(openPosition);});
    }
}
