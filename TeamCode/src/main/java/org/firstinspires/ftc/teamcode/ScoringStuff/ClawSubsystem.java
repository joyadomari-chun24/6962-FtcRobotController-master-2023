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

@Config
public class ClawSubsystem extends SubsystemBase
{
    private Servo claw;
    private ColorRangeSensor colorSensor;
    private static double closedPosition = 0.16;
    private static double openPosition = 0.4;
    public static boolean autoClosing = false;

    public static double leftClawValue = 0.955;

    public static double rightClawValue = 0.97;
    private static boolean isOpen = true;
    public ClawSubsystem(Servo theClaw, boolean isLeftClaw, ColorRangeSensor theColorSensor)
    {
        claw = theClaw;
        closedPosition = isLeftClaw ? closedPosition: leftClawValue-closedPosition;
        openPosition = isLeftClaw ? openPosition: rightClawValue-openPosition;
        colorSensor = theColorSensor;
    }
//    public void schedule(TimerTask , long delay, long period)
    @Override
    public void periodic()
    {
        super.periodic();

        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                isOpen = false;
            }
        };

        Timer timer = new Timer();

        if (colorSensor.getDistance(DistanceUnit.INCH) < 0.5 && autoClosing && isOpen && !ArmSubsystem.isScoring) {

            claw.setPosition(closedPosition);

            timer.scheduleAtFixedRate(task, 2000, 0);
            }
        }
    

    public Command closeClaw()
    {
        return new InstantCommand(() -> {claw.setPosition(closedPosition); isOpen = false;});
    }

    public Command openClaw()
    {
        return new InstantCommand(() -> {claw.setPosition(openPosition); isOpen = true;});
    }
}
