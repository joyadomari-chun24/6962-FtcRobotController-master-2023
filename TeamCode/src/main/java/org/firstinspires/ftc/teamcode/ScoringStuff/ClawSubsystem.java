package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import java.util.TimerTask;
import java.util.Timer;
import java.lang.Object;
import static java.util.concurrent.TimeUnit.*;
import org.firstinspires.ftc.teamcode.ScoringStuff.ArmSubsystem;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

////    public void schedule(TimerTask , long delay, long period)

@Config
public class ClawSubsystem extends SubsystemBase
{
    private Servo claw;
    private ColorRangeSensor colorSensor;
    private double closedPosition = 0.275;
    private double openPosition = 0.37;
    public boolean autoClosing = false;
    public static double rightClawOffset = 0.98;
    private boolean isOpen = true;
    public ClawSubsystem(Servo theClaw, boolean isLeftClaw, ColorRangeSensor theColorSensor)
    {
        claw = theClaw;
        closedPosition = isLeftClaw ? closedPosition: rightClawOffset-closedPosition;
        openPosition = isLeftClaw ? openPosition: rightClawOffset-openPosition;
        colorSensor = theColorSensor;
    }
//    @Override
//    public void periodic()
//    {
//        super.periodic();
//        if (colorSensor.getDistance(DistanceUnit.INCH) < 0.5 && autoClosing && isOpen) {
//                claw.setPosition(closedPosition);
//                isOpen = false;
//        }
//    }
    //    @Override
    public void periodic()
    {
        super.periodic();

        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                isOpen = isOpen;
            }
        };

        Timer timer = new Timer();

        if (colorSensor.getDistance(DistanceUnit.INCH) < 0.5 && autoClosing && isOpen) {

            claw.setPosition(closedPosition);
            isOpen = false;
            timer.scheduleAtFixedRate(task, 1000, 1);
        }
    }

    public Command closeClaw()
    {
        return new InstantCommand(() -> {isOpen = false; claw.setPosition(closedPosition);});
    }

    public Command openClaw()
    {
        return new InstantCommand(() -> {isOpen = true; claw.setPosition(openPosition); });
    }

    public boolean getOpen()
    {
        return isOpen;
    }
}
