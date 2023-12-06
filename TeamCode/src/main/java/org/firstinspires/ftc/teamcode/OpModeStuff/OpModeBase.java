package org.firstinspires.ftc.teamcode.OpModeStuff;

import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
import org.firstinspires.ftc.teamcode.roadrunner.util.RegressionUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
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
    protected DcMotorEx scoringSlideMotorL, scoringSlideMotorR;
    protected DcMotorEx leftFrontCurrentReader, leftRearCurrentReader, rightRearCurrentReader, rightFrontCurrentReader;
    protected NavxMicroNavigationSensor navxMicro;
    protected DistanceSensor distanceSensor;
    protected ColorRangeSensor colorSensorR, colorSensorL;
    protected SampleMecanumDrive roadrunnerMecanumDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected ScoringSlideSubsystem scoringSlides;
    protected MecanumDriveSubsystem mecanumDrive;
    protected Servo droneServo, ArmServoL, ArmServoR, hangServoL, hangServoR;
    protected Servo clawServoL, clawServoR, wristServo, leftPlatformServo, rightPlatformServo;
    protected ClawSubsystem clawL;
    protected ClawSubsystem clawR;
    protected ArmSubsystem arm;
    protected DroneLaunchSubsystem launcher;
    protected HangSubsystem hang;
    protected NavxManager gyroManager;
    ElapsedTime navxCalibrationTimer = new ElapsedTime();

    //here are the camera variables:
    public boolean targetFound;
    private AprilTagDetection detectedTag;
    public int redTagId = 8, blueTagId = 9;
    protected AprilTagProcessor aprilProcessor;
    protected PropDetectionProcessor colorProcessor = new PropDetectionProcessor(false);
    public VisionPortal aprilPortal, colorPortal;
    VisionPortal.Builder visionPortalBuilder;
    protected double tagTargetDistance = 8.0;
    protected double aprilDriveGain = 0.02;
    protected double aprilTurnGain = 0.01;
    protected double aprilStrafeGain = 0.015;
    protected double maxAprilPower = 0.5;
    protected double aprilDrive = 0;
    protected double aprilTurn = 0;
    protected double aprilStrafe = 0;
    private int colorPortalId;
    private int aprilPortalId;
    protected double maxCurrent = 99;
    protected double pastCurrent;
    public boolean currentSpike;

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
        * Todo: Everything needs to be redone! It doesn't match for some reason!
        *
        * Motors + deadwheels
        * Fl/Re - Ctl 2?
        * Bl/Le - Ctl 0?
        * Br/Fe - Ctl 3?
        * Fr - Ctl 1?
        *
        * Slides
        * Left Scoring - Exp 0?
        * Right Scoring - Exp 1?
        *
        * Scoring
        * Wrist - Exp 2
        * Left Arm - Ctl 4?
        * Right Arm - Exp 0
        * Left Claw - Ctl 3
        * Right Claw - Exp 4
        *
        * Sensors
        * NavX - Ctl I2C 1?
        * Distance Sensor - Ctl I2C ?
        *
        * Endgame
        * Drone Launcher - Exp 5
        * Left Hang - Ctl 1?
        * Right Hang - Exp 3
        *
        * */
        leftFront = new MotorEx(hardwareMap, "Fl/Re");
        leftRearLeftEncoder = new MotorEx(hardwareMap, "Bl/Le");
        rightRearFrontEncoder = new MotorEx(hardwareMap, "Br/Fe");
        rightFront = new MotorEx(hardwareMap, "Fr");
        leftFrontCurrentReader = hardwareMap.get(DcMotorEx.class, "Fl/Re");
        leftRearCurrentReader = hardwareMap.get(DcMotorEx.class, "Bl/Le");
        rightRearCurrentReader = hardwareMap.get(DcMotorEx.class, "Br/Fe");
        rightFrontCurrentReader = hardwareMap.get(DcMotorEx.class, "Fr");
        scoringSlideMotorL = hardwareMap.get(DcMotorEx.class, "scoreSlideLeft");
        scoringSlideMotorR = hardwareMap.get(DcMotorEx.class, "scoreSlideRight");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        colorSensorL = hardwareMap.get(ColorRangeSensor.class, "colorSensorL");
        colorSensorR = hardwareMap.get(ColorRangeSensor.class, "colorSensorR");
        droneServo = hardwareMap.get(Servo.class, "droneLauncher");
        hangServoL = hardwareMap.get(Servo.class, "leftHangServo");
        hangServoR = hardwareMap.get(Servo.class, "rightHangServo");
        wristServo = hardwareMap.get(Servo.class, "platformServo");
        clawServoL = hardwareMap.get(Servo.class, "leftClaw");
        clawServoR = hardwareMap.get(Servo.class, "rightClaw");
        leftPlatformServo = hardwareMap.servo.get("leftPlatformServo");
        rightPlatformServo = hardwareMap.servo.get("rightPlatformServo");
        scoringSlideMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        scoringSlideMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

//        DcMotorEx asdf = (DcMotorEx)leftFront;
//        telemetry.addData("CurrentAlert ", asdf.getCurrentAlert(CurrentUnit.AMPS));

        //Initialize subsystems
        mecanumDrive = new MecanumDriveSubsystem(leftFront, leftRearLeftEncoder, rightFront, rightRearFrontEncoder, navxMicro, distanceSensor);
        scoringSlides = new ScoringSlideSubsystem(scoringSlideMotorL, scoringSlideMotorR, false, true, telemetry);
        roadrunnerMecanumDrive = new SampleMecanumDrive(hardwareMap);
        clawL = new ClawSubsystem(clawServoL, true, colorSensorL, leftPlatformServo);
        clawR = new ClawSubsystem(clawServoR, false, colorSensorR, leftPlatformServo);
        arm = new ArmSubsystem(leftPlatformServo, rightPlatformServo, wristServo);
        launcher = new DroneLaunchSubsystem(droneServo);
        hang = new HangSubsystem(hangServoL, hangServoR);
        gyroManager = new NavxManager(navxMicro);

        //Put drone launcher in set position
        launcher.setDrone();

        //Initialize processors
        initMultiPortals();
        initProcessors();
        //https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/vision_multiportal/vision-multiportal.html

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

    //Put methods here:

    //Set camera exposure to minimize motion blur (6 ms exposure, 250 gain)
    public void setAprilExposure(VisionPortal portal)
    {
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual)
        {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        gainControl.setGain(250);
    }

    //Here are the computer vision methods:
    public void initMultiPortals() {
        List myPortalsList;

        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        colorPortalId = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        aprilPortalId = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
    }

    //Initializes processors + portals; PropDetectionProcessor has to be called elsewhere because it depends on the OpMode
    public void initProcessors() {
        //colorProcessor = new PropDetectionProcessor(false);
        AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilProcessor = aprilTagProcessorBuilder.build();
        makeColorPortal("colorCam", colorPortalId, colorProcessor);
        makeAprilPortal("aprilCam", aprilPortalId, aprilProcessor);
    }

    private void makeColorPortal(String name, int portalId, VisionProcessor processor) {
        visionPortalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, name))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(processor).setLiveViewContainerId(portalId);
        colorPortal = visionPortalBuilder.build();
    }
    private void makeAprilPortal(String name, int portalId, VisionProcessor processor) {
        visionPortalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, name))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(processor).setLiveViewContainerId(portalId);
        aprilPortal = visionPortalBuilder.build();
    }

    //Supplies a specific apriltag detection from webcam. This code is from the AprilTag example
    public void aprilTagDetect(int targetTag)
    {
        targetFound = false;
        detectedTag = null;

        List<AprilTagDetection> currentDetections = aprilProcessor.getDetections();

        telemetry.addLine("Scanning for april tags...");

        for (AprilTagDetection detection : currentDetections)
        {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null)
            {
                //  Check to see if we want to track towards this tag.
                if (detection.id == targetTag)
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
    //Rewrite this.
    public void driveUntilAprilTag(int targetTag, int timeoutInSeconds)
    {
        navxCalibrationTimer.reset();
        int timestamp = (int) navxCalibrationTimer.seconds();
        do
        {
            driveToAprilTag(targetTag);
        } while(/*aprilDrive > 0.1 || aprilTurn > 0.1 || aprilStrafe > 0.1 &&*/ navxCalibrationTimer.seconds() - timestamp < timeoutInSeconds);
    }

    //temp telemetry method: taken from double vision example
    private void AprilTag_telemetry_for_Portal_1() {
        List<AprilTagDetection> myAprilTagDetections_1;
        AprilTagDetection thisDetection_1;

        // Get a list of AprilTag detections.
        myAprilTagDetections_1 = aprilProcessor.getDetections();
        telemetry.addData("Portal 1 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_1));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection thisDetection_1_item : myAprilTagDetections_1) {
            thisDetection_1 = thisDetection_1_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (thisDetection_1.metadata != null) {
                telemetry.addLine("==== (ID " + thisDetection_1.id + ") " + thisDetection_1.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_1.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_1.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.roll, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_1.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + thisDetection_1.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_1.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_1.center.y, 6, 0) + " (pixels)");
            }
        }
    }

    @Override
    public void run() {
        super.run();
        AprilTag_telemetry_for_Portal_1();

        //Every 4 seconds, if a current reading from the past and the current reading right now is over the limit, there's a spike
        if ( (int)time % 4 == 0 && rightFrontCurrentReader.getCurrent(CurrentUnit.AMPS) > maxCurrent && pastCurrent > maxCurrent)
            currentSpike = true;
        else
            currentSpike = false;
        //Every 2 seconds, the pastCurrent gets reset
        if( ((int)time) % 2 == 0)
            pastCurrent = rightFrontCurrentReader.getCurrent(CurrentUnit.AMPS);

    }
}
