package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

@Config
public class ArmSubsystem extends SubsystemBase
{
    //Arm positions
    public static double left_pickupFront = 1; // arm is lowered, claw is parallel to ground
    public static double left_transport = 0.339; // arm is parallel to ground, claw is pointing straight up
    public static double left_deployFront = 0.659; // arm is raised, claw parallel to scoring board
    public static double left_deployBack = 0.34; // arm is parallel to slides, claw is parallel to ground
    public static double left_pickupStack = 0.85;
    public static double left_scoreYellow = 0.7;
    //Wrist positions
    public static double w_pickupFront = 0.463;
    public static double w_transport = 0.252;
    public static double w_deployFront = 0.373;
    public static double w_deployBack = 0.19;
    public static double w_pickupStack = 0.4;
    public static double w_scoreYellow = 0.35;

    // incremental arm and wrist values
    private double armIncrement = 0.020;
    private double wristIncrement = 0.01;

    //Wrist limit
    private double w_upperLimit = 1;
    private double w_lowerLimit = 0;

    private Servo ArmL, ArmR, servoForWrist;
    public ArmSubsystem(Servo leftServo, Servo rightServo, Servo wristServo)
    {
        ArmL = leftServo;
        ArmR = rightServo;
        servoForWrist = wristServo;
    }

    private void moveArm(double leftVal, double rightVal)
    {
        ArmL.setPosition(leftVal);
        ArmR.setPosition(rightVal);
    }

    private void moveWrist(double position)
    {
        servoForWrist.setPosition(position);
    }

    public Command pickupFront()
    {
        return new InstantCommand(() -> {moveArm(left_pickupFront, 1-left_pickupFront); moveWrist(w_pickupFront);});
    }

    public Command transport()
    {
        return new InstantCommand(() -> {moveArm(left_transport, 1-left_transport); moveWrist(w_transport);});
    }

    public Command deployFront()
    {
        return new InstantCommand(() -> {moveArm(left_deployFront, 1-left_deployFront); moveWrist(w_deployFront);});
    }

    public Command deployBack()
    {
        return new InstantCommand(() -> {moveArm(left_deployBack, 1-left_deployBack); moveWrist(w_deployBack);});
    }

    public Command pickupStack()
    {
        return new InstantCommand(() -> {moveArm(left_pickupStack, 1-left_pickupStack); moveWrist(w_pickupStack);});
    }

    public Command scoreYellow()
    {
        return new InstantCommand(() -> {moveArm(left_scoreYellow, 1-left_scoreYellow); moveWrist(w_scoreYellow);});
    }

    public Command incrementalWrist(int sign)
    {
        return  new InstantCommand(() -> {
                moveWrist(servoForWrist.getPosition() + sign * wristIncrement);
            }, this);
    }

    public Command incrementalArm(int sign)
    {
        return new InstantCommand(() -> {
            moveArm(ArmL.getPosition() + sign * armIncrement, ArmR.getPosition() - sign * armIncrement);
        }, this);
    }
}
