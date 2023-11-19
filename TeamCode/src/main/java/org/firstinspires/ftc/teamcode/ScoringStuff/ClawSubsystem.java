package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase
{
    private Servo claw;
    private double closedPosition = 0.15;
    private double openPosition = 0.4;
    public ClawSubsystem(Servo theClaw, boolean isLeftClaw)
    {
        claw = theClaw;
        closedPosition = isLeftClaw ? closedPosition: 1-closedPosition;
        openPosition = isLeftClaw ? openPosition: 1-openPosition;
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
