package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ClawSubsystem extends SubsystemBase
{
    private Servo claw;
    private ColorRangeSensor colorSensor;
    private double closedPosition = 0.15;
    private double openPosition = 0.4;
    public ClawSubsystem(Servo theClaw, boolean isLeftClaw, ColorRangeSensor theColorSensor)
    {
        claw = theClaw;
        closedPosition = isLeftClaw ? closedPosition: 1-closedPosition;
        openPosition = isLeftClaw ? openPosition: 1-openPosition;
        colorSensor = theColorSensor;
    }

    @Override
    public void periodic()
    {
        super.periodic();
        if (colorSensor.getDistance(DistanceUnit.INCH) < 0.5) {
            closeClaw();
        }
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
