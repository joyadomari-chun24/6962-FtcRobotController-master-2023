package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunnerStuff.drive.SampleMecanumDrive;

public class OpModeBase extends CommandOpMode
{
    /**
     *
     * This is the file with all the basic stuff for any OpMode (TeleOp or auto). Simply extend
     * this when making a new OpMode so that you don't have to write all of this every time and go
     * through every single file to make a change.
     *
     * */
    protected DcMotorEx leftFront, leftRearLeftEncoder, rightRearFrontEncoder, rightFront, scoringSlides, intakeSlides;
    protected NavxMicroNavigationSensor navxMicro;
    protected DistanceSensor distanceSensor;
    protected SampleMecanumDrive roadrunnerMecanumDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize()
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "Fl/Re");
        leftRearLeftEncoder = hardwareMap.get(DcMotorEx.class, "Bl/Le");
        rightRearFrontEncoder = hardwareMap.get(DcMotorEx.class, "Br/Fe");
        rightFront = hardwareMap.get(DcMotorEx.class, "Fr");
        scoringSlides = hardwareMap.get(DcMotorEx.class, "Score");
        intakeSlides = hardwareMap.get(DcMotorEx.class, "Intake");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        scoringSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        scoringSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        scoringSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //(Declare vision pipelines here)

        //(Initialize subsystems here)

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
        }
        telemetry.log().clear(); telemetry.log().add("Initialization Complete. Good luck!");
        telemetry.clear(); telemetry.update();
    }

    //(Put methods and stuff here)
}
