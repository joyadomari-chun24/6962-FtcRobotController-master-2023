package org.firstinspires.ftc.teamcode.ScoringStuff;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase
{
    //Servo positions
    public static double left_pickupFront = 0.1;
    public static double left_topDownFront = 0.3;
    public static double left_deployFront = 0.4;
    public static double left_deployBack = 0.6;

    public static double w_topDownFront = 0.6;
    public static double w_pickupFront = 0.2;
    public static double w_deployBack = 0;
    public static double w_deployFront = 0;

    // incremental arm and wrist values
    private double armIncrement = 0.025;
    private double wristIncrement = 0.01;

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

    //Debug method
    public Command positionW()
    {
        return new InstantCommand(() -> {servoForWrist.setPosition(w_pickupFront);});
    }

    public Command pickupFront()
    {
        return new InstantCommand(() -> {moveArm(left_pickupFront, 1-left_pickupFront); moveWrist(w_pickupFront);});
    }

    public Command topDown()
    {
        return new InstantCommand(() -> {moveArm(left_topDownFront, 1-left_topDownFront); moveWrist(w_topDownFront);});
    }

    public Command deployFront()
    {
        return new InstantCommand(() -> {moveArm(left_deployFront, 1-left_deployFront); moveWrist(w_deployFront);});
    }

    public Command deployBack()
    {
        return new InstantCommand(() -> {moveArm(left_deployBack, 1-left_deployBack); moveWrist(w_deployBack);});
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
