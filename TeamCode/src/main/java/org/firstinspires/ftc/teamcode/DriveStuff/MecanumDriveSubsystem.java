package org.firstinspires.ftc.teamcode.DriveStuff;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase
{
    private int slowModeFactor = 2;
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kF = 0;
    private final MecanumDrive drive;
    private DistanceSensor distSensor;
    private boolean aligningToBackdrop = false;
    private double aligningInches = 4;

    private ElapsedTime time = new ElapsedTime();
    public Telemetry telemetry;

    public MecanumDriveSubsystem(MotorEx FL, MotorEx BL, MotorEx FR, MotorEx BR, NavxMicroNavigationSensor navxSensor, DistanceSensor distanceSensor)
    {
        navx = navxSensor;
        FL.setInverted(true);
        BL.setInverted(true);
        drive = new MecanumDrive(false, FL, FR, BL, BR);
        distSensor = distanceSensor;
    }

    @Override
    public void periodic()
    {
        super.periodic();
    }

    /**
     * Then put methods here:
     * */

    //I don't know why it needs a DoubleSupplier instead of just a double. I think it's just because the gamepad gives them.
    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier navxAngle, Telemetry infoThing)
    {
        return new RunCommand(() -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble() * 0.75, navxAngle.getAsDouble()), this);
    }

    //'Telemetry' is a parameter so that I can print telemetry from this method for debugging
    public Command slowFieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier navxAngle, Telemetry infoThing2)
    {
        return new RunCommand(() -> drive.driveFieldCentric(strafeSpeed.getAsDouble()/slowModeFactor, forwardSpeed.getAsDouble()/slowModeFactor, turnSpeed.getAsDouble()/slowModeFactor, navxAngle.getAsDouble()), this);
    }

    //robot centric drive for the april tags
    public Command roboCentric(double strafeSpeed, double forwardSpeed, double turnSpeed)
    {
        return new RunCommand(() -> drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed), this);
    }

    public Command alignToBackdrop()
    {
        if (distSensor.getDistance(DistanceUnit.INCH) > aligningInches)
        {
            return new InstantCommand(()->drive.driveRobotCentric(0, -1, 0));
        }
        else if (distSensor.getDistance(DistanceUnit.INCH) < aligningInches)
        {
            return new InstantCommand(() -> drive.driveRobotCentric(0, 1, 0));
        }
        return new InstantCommand(() -> drive.driveRobotCentric(0, 0, 0));
    }

    //Deprecated method. Might remove depending on if itʻs usefull in auto
    public void setBackdropAlignment(boolean state)
    {
        aligningToBackdrop = state;
    }
}