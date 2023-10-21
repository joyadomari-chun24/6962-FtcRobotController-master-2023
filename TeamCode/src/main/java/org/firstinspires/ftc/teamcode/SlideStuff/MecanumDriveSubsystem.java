package org.firstinspires.ftc.teamcode.SlideStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class MecanumDriveSubsystem extends SubsystemBase
{
    private NavxMicroNavigationSensor navx;
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kF = 0;
    private ElapsedTime time = new ElapsedTime();

    public MecanumDriveSubsystem()
    {
        //
    }

    //(Then just put commands here I guess)
}
