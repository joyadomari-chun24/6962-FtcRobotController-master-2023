package org.firstinspires.ftc.teamcode.demonstrations;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "W_MultiPortalskeleton")
public class multiportalSkeleton extends LinearOpMode {

  VisionPortal.Builder myVisionPortalBuilder;
  int Portal_1_View_ID;
  int Portal_2_View_ID;
  AprilTagProcessor myAprilTagProcessor_1;
  VisionProcessor myAprilTagProcessor_2;
  VisionPortal myVisionPortal_1;
  VisionPortal myVisionPortal_2;

  /**
   * Describe this function...
   */
  private void initMultiPortals() {
    List myPortalsList;

    myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
    Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
    Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
    while (!gamepad1.y && opModeInInit()) {}
  }
  @Override
  public void runOpMode() {
    initMultiPortals();
    initAprilTag();
    waitForStart();
  }

  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Create each AprilTagProcessor by calling build.
    myAprilTagProcessor_1 = myAprilTagProcessorBuilder.build();
    myAprilTagProcessor_2 = myAprilTagProcessorBuilder.build();
    Make_first_VisionPortal("colorCam", Portal_1_View_ID, myAprilTagProcessor_1);
    Make_second_VisionPortal("aprilCam", Portal_2_View_ID, myAprilTagProcessor_2);
  }

  private void Make_first_VisionPortal(String name, int portalId, VisionProcessor processor) {
    myVisionPortalBuilder = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, name))
            .setCameraResolution(new Size(320, 240))
            .addProcessor(processor).setLiveViewContainerId(portalId);
    myVisionPortal_1 = myVisionPortalBuilder.build();
  }
  private void Make_second_VisionPortal(String name, int portalId, VisionProcessor processor) {
    myVisionPortalBuilder = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, name))
            .setCameraResolution(new Size(320, 240))
            .addProcessor(processor).setLiveViewContainerId(portalId);
    myVisionPortal_2 = myVisionPortalBuilder.build();
  }
}