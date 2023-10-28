package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class ArmSubsystem
{
    //Servo positions
    public static double left_pickupFront = 0.32;
    public static double left_topDownFront = 0.71;
    public static double left_deployFront = 0.9;
    public static double left_deployBack = 1;

    public static double w_topDownFront = 0.6;
    public static double w_pickupFront = 0.2;
    public static double w_deployBack = 0;
    //manually adjustable, so no need
    //public static double w_deployFront = 0;
    private Servo ArmL, ArmR, servoForWrist;
    public ArmSubsystem(Servo leftServo, Servo rightServo, Servo wristServo)
    {
        ArmL = leftServo;
        ArmR = rightServo;
        servoForWrist = wristServo;
    }

    private void moveArm(double leftVal)
    {
        ArmL.setPosition(leftVal);
        ArmR.setPosition(1-leftVal);
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
        return new InstantCommand(() -> {moveArm(left_pickupFront); moveWrist(w_pickupFront);});
    }

    public Command topDown()
    {
        return new InstantCommand(() -> {moveArm(left_topDownFront); moveWrist(w_topDownFront);});
    }

    public Command deployFront()
    {
        return new InstantCommand(() -> {moveArm(left_deployFront);});
        //return new InstantCommand(() -> {moveArm(left_deployFront); moveWrist(w_deployFront);});
    }

    public Command deployBack()
    {
        return new InstantCommand(() -> {moveArm(left_deployBack); moveWrist(w_deployBack);});
    }

    public Command incrementalWrist(double increment)
    {
        return  (servoForWrist.getPosition() + increment > 0 && servoForWrist.getPosition() < 1) ? new InstantCommand(() -> {moveWrist(servoForWrist.getPosition() + increment);}) : null;
    }

    public Command incrementalArm(double increment)
    {
        return (ArmL.getPosition() + increment > 0 && ArmL.getPosition() < 1) ? new InstantCommand(() -> {moveArm(ArmL.getPosition() + increment);}) : null;
    }
}
