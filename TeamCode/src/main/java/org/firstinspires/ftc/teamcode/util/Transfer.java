public class Transfer extends LinearOpMode {
//package org.firstinspires.ftc.teamcode.util;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//public class Transfer extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        waitForStart();
//
//        double tgtPower = 0;
//
//        while (opModeIsActive()) {
//            CRServo leftFlipServo = hardwareMap.crservo.get("leftFlipServo");
//            CRServo rightFlipServo = hardwareMap.crservo.get("rightFlipServo");
//            CRServo leftIntakeServo = hardwareMap.crservo.get("leftIntakeServo");
//            CRServo rightIntakeServo = hardwareMap.crservo.get("rightIntakeServo");
//
//            double leftFlipServoPos = leftFlipServo.getPosition();
//            double rightFlipServoPos = rightFlipServo.getPosition();
//            double leftIntakeServoPos = leftIntakeServo.getPosition();
//            double rightIntakeServoPos = rightIntakeServo.getPosition();
//
//            // check if button pressed
//            if(gamepad1.y) {
//                leftFlipServo.setPower(-1);
//                rightFlipServo.setPower(1);
//
//            } else if (gamepad1.x) {
//                leftFlipServo.setPower(-1);
//                rightFlipServo.setPower(1);
//            }
//            telemetry.addData("Servo Position", servoTest.getPosition());
//            telemetry.addData("Target Power", tgtPower);
//            telemetry.addData("Motor Power", motorTest.getPower());
//            telemetry.addData("Status", "Running");
//            telemetry.update();
//
//        }
//    }
//
//
//}