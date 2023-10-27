package org.firstinspires.ftc.teamcode.opModeStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.demonstrations.VisionPortalStreamingOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

//There should probably be an auto for each of the four starting positions on the field and
@Autonomous(name = "CloseToBackDropLeftOrSomethingLikeThat")
public class RapidAutoOp extends OpModeBase
{
    /**
     * This is a very hastily made opmode that is bad. For comp 1 only.
     *
     * Just copy this file for each opmode and change the name.
     *
     * Use FTC dashboard to change red/blue boolean in between games so that it detects the right color (is under VisionPortalStreamingOpMode)
     */

    double leftZone = 100;
    double rightZone = 500;

    //Everything goes in here
    @Override
    public void initialize()
    {
        super.initialize();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.log().add("This auto has initialized!");
        waitForStart();


        //VISION STUFF:

        //Use FTC dashboard to change red/blue boolean in between games so that it detects the right color (is under VisionPortalStreamingOpMode)

        //Creates an instance of the processor from the demonstration
        final VisionPortalStreamingOpMode.CameraStreamProcessor processor = new VisionPortalStreamingOpMode.CameraStreamProcessor();

        //Builds a vision portal with the processor we just made and sets the webcam (if we use .addProcessors, we can add multiple processors to the frame. See double vision example)
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        waitForStart();

        /**
         * GetPropLocation(processor.largestContourX, processor.largestContourY)
         * returns LEFT, RIGHT, or CENTER depending on where the prop is.
         * Use this method to figure out what the location to drop off the pixel is.
         *
         * This code below only does the telemetry as a demonstration. You will need to write something
         */
        telemetry.addData("Prop Location", GetPropLocation(processor.largestContourX, processor.largestContourY));
        telemetry.update();

        //Change the x, y, heading to what the robot will be started at, or not. If you don't and this stays at 0, 0, 0, just know that the robot's positon will start at 0,0.
        //This sets the location of the bot on the field.
        roadrunnerMecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        //Set the trajectory here. For example. Something like this:
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10).build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .forward(5).build();

        //Then get the bot to run the path.
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }


    //Returns the prop location. Literally copied from the other thing. Like I said, this OpMode is bad and should not be reused.
    public String GetPropLocation(double contourX, double contourY)
    {
        if(contourX < leftZone)
            return "LEFT";
        else if (contourX > rightZone)
            return "RIGHT";
        else
            return "CENTER";
    }
}
