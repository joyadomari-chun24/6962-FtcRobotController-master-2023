package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase
{
    private NavxMicroNavigationSensor navx;
    private int slowModeFactor = 2;
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kF = 0;
    private final MecanumDrive drive;
    private ElapsedTime time = new ElapsedTime();
    public Telemetry telemetry;

    public MecanumDriveSubsystem(MotorEx FL, MotorEx BL, MotorEx FR, MotorEx BR, NavxMicroNavigationSensor navxSensor)
    {
        navx = navxSensor;
        FL.setInverted(true);
        BL.setInverted(true);
        drive = new MecanumDrive(false, FL, FR, BL, BR);
    }

    /**
     * Then put commands here:
     * */

    //to be honest, I don't know why it needs a DoubleSupplier instead of just a double. I think it's just because the gamepad gives them.
    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, double navxAngle, Telemetry infoThing)
    {
        telemetry = infoThing;
        telemetry.addLine("Driving field oriented!");
        return new RunCommand(() -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble(), navxAngle), this);
    }

    //A test from discord
    public void fieldCentric2(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, double navxAngle)
    {
        drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble(), navxAngle);
    }

    public Command slowFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double navxAngle, Telemetry infoThing2)
    {
        telemetry = infoThing2;
        telemetry.addLine("Driving slow!");
        return new RunCommand(() -> drive.driveFieldCentric(strafeSpeed/slowModeFactor, forwardSpeed/slowModeFactor, turnSpeed/slowModeFactor, navxAngle), this);
    }
}
