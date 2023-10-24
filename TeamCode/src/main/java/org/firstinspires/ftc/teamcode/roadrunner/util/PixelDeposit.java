package org.firstinspires.ftc.teamcode.roadrunner.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;// switch to iterative later
import com.qualcomm.robotcore.hardware.Servo;

public class PixelDeposit extends LinearOpMode { // switch to iterative later

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        Servo platformServo = hardwareMap.servo.get("platformServo");

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // rotates platform to scoring
                platformServo.setPosition(0); //change value based on tuning
            } else if (gamepad1.b) {
                // rotates platform back in
                platformServo.setPosition(1); //change value based on tuning
            }
        }
    }
}