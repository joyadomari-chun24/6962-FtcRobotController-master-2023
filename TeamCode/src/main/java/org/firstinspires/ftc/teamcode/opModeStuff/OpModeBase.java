package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.SlideStuff.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.SlideStuff.ScoringSlideSubsystem;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionPipeline;
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
    protected MotorEx leftFront, leftRearLeftEncoder, rightRearFrontEncoder, rightFront;
    protected DcMotorEx scoringSlideMotor, intakeSlideMotor;
    protected NavxMicroNavigationSensor navxMicro;
    protected DistanceSensor distanceSensor;
    protected SampleMecanumDrive roadrunnerMecanumDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected IntakeSlideSubsystem intakeSlides;
    protected ScoringSlideSubsystem outtakeSlides;
    protected MecanumDriveSubsystem mecanumDrive;
    protected PropDetectionPipeline propDetectionPipeline;
    protected Servo droneServo, ArmServoL, ArmServoR;
    protected Servo clawServo, wristServo;
    protected ClawSubsystem claw;

    ElapsedTime navxCalibrationTimer = new ElapsedTime();

    @Override
    public void initialize()
    {
        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        leftFront = new MotorEx(hardwareMap, "Fl/Re");
        leftRearLeftEncoder = new MotorEx(hardwareMap, "Bl/Le");
        rightRearFrontEncoder = new MotorEx(hardwareMap, "Br/Fe");
        rightFront = new MotorEx(hardwareMap, "Fr");
        scoringSlideMotor = hardwareMap.get(DcMotorEx.class, "Score");
        //intakeSlideMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        //wristServo = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");
        /*scoringSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        scoringSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);*/
        scoringSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //Declare vision pipelines here
        //propDetectionPipeline = new PropDetectionPipeline();

        //Initialize subsystems
        mecanumDrive = new MecanumDriveSubsystem(leftFront, leftRearLeftEncoder, rightFront, rightRearFrontEncoder, navxMicro);
        //intakeSlides = new IntakeSlideSubsystem(intakeSlideMotor);
        outtakeSlides = new ScoringSlideSubsystem(scoringSlideMotor);
        roadrunnerMecanumDrive = new SampleMecanumDrive(hardwareMap);
        claw = new ClawSubsystem(clawServo);


        //roadrunnerMecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Wait until the gyro calibration is complete
        navxCalibrationTimer.reset();
        while (navxMicro.isCalibrating())
        {
            telemetry.addData("calibrating", "%s", Math.round(navxCalibrationTimer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
        }
        telemetry.log().clear();
        telemetry.log().add("Base Initialization Complete. Good luck!");
        telemetry.clear();
        telemetry.update();
    }

    //(Put methods and stuff here)
}
