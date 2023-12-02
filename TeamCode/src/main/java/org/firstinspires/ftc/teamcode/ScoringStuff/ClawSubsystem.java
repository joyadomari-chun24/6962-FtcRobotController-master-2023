package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.TimerTask;
import java.util.Timer;
import java.lang.Object;
import static java.util.concurrent.TimeUnit.*;
import org.firstinspires.ftc.teamcode.ScoringStuff.ArmSubsystem;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ClawSubsystem extends SubsystemBase
{
    private Servo claw, ArmL;
    private ColorRangeSensor colorSensor;
    private double closedPosition = 0.275;
    private double openPosition = 0.5;
    public boolean autoClosing = false;
    public static double rightClawOffset = 0.98;
    private boolean isOpen = true;
    private double detectionInches = 0.8;
    private ElapsedTime colorSensorTimer = new ElapsedTime();
    public ClawSubsystem(Servo theClaw, boolean isLeftClaw, ColorRangeSensor theColorSensor, Servo leftArm)
    {
        claw = theClaw;
        closedPosition = isLeftClaw ? closedPosition: rightClawOffset-closedPosition;
        openPosition = isLeftClaw ? openPosition: rightClawOffset-openPosition;
        colorSensor = theColorSensor;
        ArmL = leftArm;
    }

    @Override
    public void periodic()
    {
        super.periodic();
        if (colorSensor.getDistance(DistanceUnit.INCH) < detectionInches && autoClosing && isOpen && ArmL.getPosition() > 0.8 && colorSensorTimer.seconds() > 1)
        {
            claw.setPosition(closedPosition);
            isOpen = false;
        }
    }

    public Command closeClaw()
    {
        return new InstantCommand(() -> {isOpen = false; claw.setPosition(closedPosition);});
    }

    public Command openClaw()
    {
        return new InstantCommand(() -> {isOpen = true; claw.setPosition(openPosition); colorSensorTimer.reset();});
    }

    public boolean getOpen()
    {
        return isOpen;
    }
}
