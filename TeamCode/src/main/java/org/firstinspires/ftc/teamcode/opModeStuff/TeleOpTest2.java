package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SlideStuff.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@TeleOp
public class TeleOpTest2 extends OpModeBase
{
    SampleMecanumDrive drive;
    @Override
    public void initialize()
    {
        super.initialize();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //(add the listeners for button inputs that lead to commands here)

        mecanumDrive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.log().add("TeleOpTest has initialized.");
        telemetry.update();
    }

    public void run()
    {
        super.run();

    }
}
