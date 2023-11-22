package org.firstinspires.ftc.teamcode.OpModeStuff;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EndgameStuff.HangSubsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.ScoringStuff.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ScoringStuff.ClawSubsystem;
import org.firstinspires.ftc.teamcode.DriveStuff.NavxManager;
import org.firstinspires.ftc.teamcode.EndgameStuff.DroneLaunchSubsystem;
//import org.firstinspires.ftc.teamcode.SlideStuff.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.DriveStuff.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.SlideStuff.ScoringSlideSubsystem;
import org.firstinspires.ftc.teamcode.VisionStuff.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

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
    protected DcMotorEx scoringSlideMotorL, scoringSlideMotorR, intakeSlideMotor;
    protected NavxMicroNavigationSensor navxMicro;
    protected DistanceSensor distanceSensor;
    protected SampleMecanumDrive roadrunnerMecanumDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    //protected IntakeSlideSubsystem intakeSlides;
    protected ScoringSlideSubsystem scoringSlides;
    protected MecanumDriveSubsystem mecanumDrive;
    protected PropDetectionProcessor propDetectionPipeline;
    protected Servo droneServo, ArmServoL, ArmServoR, hangServoL, hangServoR;
    protected Servo clawServoL, clawServoR, wristServo, leftPlatformServo, rightPlatformServo;
    protected ClawSubsystem clawL;
    protected ClawSubsystem clawR;
    protected ArmSubsystem arm;
    protected DroneLaunchSubsystem launcher;
    protected HangSubsystem hang;
    protected NavxManager gyroManager;
    public int blueLeftAprilID = 1, blueCenterAprilID = 2, blueRightAprilID = 3, redLeftAprilID = 4, redCenterAprilID = 5, redRightAprilID = 6;
    protected AprilTagProcessor backdropAprilTag;
    boolean targetFound;
    AprilTagDetection detectedTag;
    VisionPortal aprilPortal;
    protected double tagTargetDistance = 12.0;
    protected double aprilDriveGain = 0.02;
    protected double aprilTurnGain = 0.01;
    protected double aprilStrafeGain = 0.015;
    protected double maxAprilPower = 0.5;
    protected double aprilDrive = 0;
    protected double aprilTurn = 0;
    protected double aprilStrafe = 0;

    ElapsedTime navxCalibrationTimer = new ElapsedTime();

    /*
    * Things with positions to be reset if needed:
    * -Claw
    * -Hang
    * -Drone launcher
    * -Wrist
    * -Arm
    * -Slide motors
    * */

    @Override
    public void initialize()
    {
        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        /*
        * Config:
        *
        * Motors + deadwheels
        * Fl/Re - Ctl 2
        * Bl/Le - Ctl 0
        * Br/Fe - Ctl 3
        * Fr - Ctl 1
        *
        * Slides
        * Left Scoring - Exp 0?
        * Right Scoring - Exp 1?
        *
        * Scoring
        * Wrist - Exp 2
        * Left Arm - Ctl 4
        * Right Arm - Exp 1
        * Left Claw - Ctl 5
        * Right Claw - Exp 4
        *
        * Sensors
        * NavX - Ctl I2C 1
        * Distance Sensor - Ctl I2C ?
        *
        * Endgame
        * Drone Launcher - Ctl 3
        * Left Hang - Ctl 1
        * Right Hang - Exp 3
        *
        * */
        leftFront = new MotorEx(hardwareMap, "Fl/Re");
        leftRearLeftEncoder = new MotorEx(hardwareMap, "Bl/Le");
        rightRearFrontEncoder = new MotorEx(hardwareMap, "Br/Fe");
        rightFront = new MotorEx(hardwareMap, "Fr");
        scoringSlideMotorL = hardwareMap.get(DcMotorEx.class, "scoreSlideLeft");
        scoringSlideMotorR = hardwareMap.get(DcMotorEx.class, "scoreSlideRight");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        droneServo = hardwareMap.get(Servo.class, "droneLauncher");
        hangServoL = hardwareMap.get(Servo.class, "leftHangServo");
        hangServoR = hardwareMap.get(Servo.class, "rightHangServo");
        wristServo = hardwareMap.get(Servo.class, "platformServo");
        clawServoL = hardwareMap.get(Servo.class, "leftClaw");
        clawServoR = hardwareMap.get(Servo.class, "rightClaw");
        leftPlatformServo = hardwareMap.servo.get("leftPlatformServo");
        rightPlatformServo = hardwareMap.servo.get("rightPlatformServo");
        /*scoringSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/
        scoringSlideMotorL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        scoringSlideMotorR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        scoringSlideMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        scoringSlideMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //April tag startup
//        backdropAprilTag = new AprilTagProcessor.Builder().build();
//        backdropAprilTag.setDecimation(2); // Higher decimation = increased performance but less distance
//        aprilPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2")).addProcessor(backdropAprilTag)
//                .build();

        //Set camera exposure to minimize motion blur (6 ms exposure, 250 gain)
//        ExposureControl exposureControl = aprilPortal.getCameraControl(ExposureControl.class);
//        if (exposureControl.getMode() != ExposureControl.Mode.Manual)
//        {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//            sleep(50);
//        }
//        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
//        sleep(20);
//        GainControl gainControl = aprilPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(250);

        //Initialize subsystems
        mecanumDrive = new MecanumDriveSubsystem(leftFront, leftRearLeftEncoder, rightFront, rightRearFrontEncoder, navxMicro);
        //intakeSlides = new IntakeSlideSubsystem(intakeSlideMotor);
        scoringSlides = new ScoringSlideSubsystem(scoringSlideMotorL, scoringSlideMotorR, telemetry);
        roadrunnerMecanumDrive = new SampleMecanumDrive(hardwareMap);
        clawL = new ClawSubsystem(clawServoL, true);
        clawR = new ClawSubsystem(clawServoR, false);
        arm = new ArmSubsystem(leftPlatformServo, rightPlatformServo, wristServo);
        launcher = new DroneLaunchSubsystem(droneServo);
        hang = new HangSubsystem(hangServoL, hangServoR);
        gyroManager = new NavxManager(navxMicro);

        //Put drone launcher in set position
        launcher.setDrone();

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

//        ExposureControl exposureControl = aprilPortal.getCameraControl(ExposureControl.class);
//        if (exposureControl.getMode() != ExposureControl.Mode.Manual)
//        {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//            sleep(50);
//        }
//        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
//        sleep(20);
//        GainControl gainControl = aprilPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(250);
    }

    //Put methods here

    //Supplies a specific apriltag detection from webcam. This code is from the AprilTag example
    public void aprilTagDetect(int targetTag)
    {
        targetFound = false;
        detectedTag = null;

        List<AprilTagDetection> currentDetections = backdropAprilTag.getDetections();

        telemetry.addLine("Scanning for april tags...");

        for (AprilTagDetection detection : currentDetections)
        {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null)
            {
                //  Check to see if we want to track towards this tag.
                if ((targetTag < 0) || (detection.id == targetTag))
                {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    telemetry.addData("Detected!", "Tag ID %d found.", detection.id);
                    break;
                }
                else
                {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            }
            else
            {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown tag: ", detection.id);
            }
        }
    }

    //Drives robot to the apriltag for the loop
    public void driveToAprilTag(int tagTarget)
    {
        aprilTagDetect(tagTarget);
        if(targetFound)
        {
            //Properties of tag
            double  rangeError      = (detectedTag.ftcPose.range - tagTargetDistance);
            double  headingError    = detectedTag.ftcPose.bearing;
            double  yawError        = detectedTag.ftcPose.yaw;

            //Properties of robot to drive
            aprilDrive = Range.clip(rangeError * aprilDriveGain, -maxAprilPower, maxAprilPower);
            aprilTurn = Range.clip(headingError * aprilTurnGain, -maxAprilPower, maxAprilPower);
            aprilStrafe = Range.clip(-yawError * aprilStrafeGain, -maxAprilPower, maxAprilPower);

            mecanumDrive.roboCentric(-aprilDrive, aprilTurn, -aprilStrafe);
        }
    }

    //VERY EXPERIMENTAL; Drives robot until it is aligned with apriltag or the timeout passes
    public void driveUntilAprilTag(int targetTag, int timeoutInSeconds)
    {
        int timestamp = (int) time;
        do
        {
            driveToAprilTag(targetTag);
        } while(aprilDrive > 0.1 || aprilTurn > 0.1 || aprilStrafe > 0.1 && time - timestamp < timeoutInSeconds);
    }
}
