package org.firstinspires.ftc.teamcode.opModeStuff;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SlideStuff.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.SlideStuff.ScoringSlideSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class OpModeBase extends CommandOpMode
{
    /**
     *
     * This is the file with all the basic stuff for any OpMode (TeleOp or auto). Simply extend
     * this when making a new OpMode so that you don't have to write all of this every time and go
     * through every single file to make a change.
     *
     * */
    protected DcMotorEx leftFront, leftRearLeftEncoder, rightRearFrontEncoder, rightFront, scoringSlideMotor, intakeSlideMotor;
    protected NavxMicroNavigationSensor navxMicro;
    protected DistanceSensor distanceSensor;
    protected SampleMecanumDrive roadrunnerMecanumDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected IntakeSlideSubsystem intakeSlides;
    protected ScoringSlideSubsystem outtakeSlides;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        leftFront = hardwareMap.get(DcMotorEx.class, "Fl/Re");
        leftRearLeftEncoder = hardwareMap.get(DcMotorEx.class, "Bl/Le");
        rightRearFrontEncoder = hardwareMap.get(DcMotorEx.class, "Br/Fe");
        rightFront = hardwareMap.get(DcMotorEx.class, "Fr");
        scoringSlideMotor = hardwareMap.get(DcMotorEx.class, "Score");
        intakeSlideMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        scoringSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        scoringSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        scoringSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //(Declare vision pipelines here)

        //Initialize subsystems
        intakeSlides = new IntakeSlideSubsystem();
        outtakeSlides = new ScoringSlideSubsystem();

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
        }
        telemetry.log().clear();
        telemetry.log().add("Initialization Complete. Good luck!");
        telemetry.clear();
        telemetry.update();
    }

    //(Put methods and stuff here)
}
