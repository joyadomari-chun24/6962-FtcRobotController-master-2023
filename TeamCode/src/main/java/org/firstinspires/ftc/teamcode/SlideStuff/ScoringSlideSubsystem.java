package org.firstinspires.ftc.teamcode.SlideStuff;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class ScoringSlideSubsystem extends SlideBaseSubsystem
{
    private DcMotorEx motorLeft, motorRight;
    private double slidePower;
    private int joystickScalar = 1;
    private int target = 0;
    public enum motorSide {LEFT, RIGHT}
    public Telemetry telemetry;
    public ScoringSlideSubsystem(DcMotorEx scoringMotorL, DcMotorEx scoringMotorR, Telemetry tel)
    {
        super(0, 0, 0, true, false, scoringMotorL, scoringMotorR);
        motorLeft = scoringMotorL;
        motorRight = scoringMotorR;
        telemetry = tel;
    }
    @Override
    public void periodic()
    {
        double powerL = PIDControl(target, motorLeft.getCurrentPosition(), motorLeft);
        double powerR = PIDControl(target, motorRight.getCurrentPosition(), motorRight);
        // First part sets motors to the joystick if the joystick is moving and sets target to it so slides stay when let go
        if (Math.abs(slidePower) > 0.05)
        {
            motorLeft.setPower(slidePower/2);
            motorRight.setPower(slidePower/2);
            target = motorLeft.getCurrentPosition() + (int) slidePower * joystickScalar;
        }
        else //If not, move slides to target (current pos from joystick or a button's set position)
        {
            motorLeft.setPower(powerL);
            motorRight.setPower(powerR);
        }
        telemetry.addData("PID Power L", powerL);
        telemetry.addData("PID Power R", powerR);
        super.periodic();
    }

    public Command extendToPosition(int targetPos)
    {
        return new InstantCommand(() -> target = targetPos);
    }

    public Command slideMovement(DoubleSupplier motorPower)
    {
        return new RunCommand(() -> {slidePower = motorPower.getAsDouble();}, this);
    }

    public int getPosition(motorSide allCapsMotorSide)
    {
        return allCapsMotorSide == motorSide.LEFT ? motorLeft.getCurrentPosition() : motorRight.getCurrentPosition();
    }
}