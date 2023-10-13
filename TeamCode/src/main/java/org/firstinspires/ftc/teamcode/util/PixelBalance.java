package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;// switch to iterative later
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelBalance extends LinearOpMode { // switch to iterative later

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        CRServo leftPlatformServo = hardwareMap.crservo.get("leftPlatformServo");
        CRServo rightPlatformServo = hardwareMap.crservo.get("rightPlatformServo");
        Servo platformServo = hardwareMap.servo.get("platformServo");

        // ALL + AND - NEED TO BE TESTED
        while (opModeIsActive()) {
            // check if button pressed and in down position
            if (gamepad1.a && (platformServo.getPosition() < 1 && platformServo.getPosition() > 0.8)) {
                // starts rotating platform
                leftPlatformServo.setPower(0.5);
                rightPlatformServo.setPower(-0.5);
                platformServo.setPosition(0);
                // counter rotates little bits at a time
                int timeToRotate = 40; // (1/timeToRotate) * 0.05 seconds

                for (int i = 0; i < timeToRotate; i++) {
                    sleep(50);
                    platformServo.setPosition(platformServo.getPosition() + 1/timeToRotate);
                }
                // stops rotating platform
                leftPlatformServo.setPower(0);
                rightPlatformServo.setPower(0);

            }
            // check if button pressed and in up position
            else if (gamepad1.b && (platformServo.getPosition() < 0.2 && platformServo.getPosition() > 0)) {
                // starts rotating platform
                leftPlatformServo.setPower(-0.5);
                rightPlatformServo.setPower(0.5);
                platformServo.setPosition(0);
                // counter rotates little bits at a time
                int timeToRotate = 40; // (1/timeToRotate) * 0.05 seconds

                for (int i = 0; i < timeToRotate; i++) {
                    sleep(50);
                    platformServo.setPosition(platformServo.getPosition() - 1/timeToRotate);
                }
                // stops rotating platform
                leftPlatformServo.setPower(0);
                rightPlatformServo.setPower(0);
            }
        }
    }
}