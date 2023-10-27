package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class ArmSubsystem
{
    public static double forwardPickupPos = 1;
    public static double topDownPickupPos = 0;
    public static double forwardDeployPos = 0;
    public static double backDeployPos = -1;

    public static double wristDebugVal = 0.5;
    private Servo ArmL, ArmR, servoForWrist;
    public ArmSubsystem(Servo LeftServo, Servo RightServo, Servo WristServo)
    {
        ArmL = LeftServo;
        ArmR = RightServo;
        servoForWrist = WristServo;
    }

    public Command position1()
    {
        return new InstantCommand(() ->
        {
            ArmR.setPosition(forwardDeployPos * -1);
            ArmL.setPosition(forwardDeployPos);
        });
    }

    public Command moveWrist()
    {
        return new InstantCommand(() -> {servoForWrist.setPosition(wristDebugVal);});
    }
}
