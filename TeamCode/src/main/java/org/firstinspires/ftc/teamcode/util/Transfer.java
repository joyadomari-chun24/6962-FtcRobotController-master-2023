package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;// switch to iterative later
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer extends LinearOpMode { // switch to iterative later

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        // finding motors, servos, and sensors in the hardware map
        Servo leftFlipServo = hardwareMap.servo.get("leftFlipServo");
        Servo rightFlipServo = hardwareMap.servo.get("rightFlipServo");
        CRServo leftIntakeServo = hardwareMap.crservo.get("leftIntakeServo");
        CRServo rightIntakeServo = hardwareMap.crservo.get("rightIntakeServo");
        Servo platformServo = hardwareMap.servo.get("platformServo");
        AnalogSensor intakeBreakBeam = hardwareMap.get(AnalogSensor.class, "intakeBreakBeam");
        AnalogSensor scoringBreakBeam = hardwareMap.get(AnalogSensor.class, "scoringBreakBeam");

        // fraction of 280° that the regular servos need to turn in order to flip
        double distanceToFlip = 0.6; // tune based on testing
        // also tune whether it's + or - distanceToFlip
        // also tune whether it's + or - setPower

        while (opModeIsActive()) {
            // check if button pressed && other requirements true
            if(gamepad1.y && // y button pressed
              (intakeBreakBeam.readRawVoltage() < 0.1) && // intake break beam blocked
              (scoringBreakBeam.readRawVoltage() < 0.1) && // scoring break beam blocked
              (platformServo.getPosition() < 1 && platformServo.getPosition() > 0.8)) // platform servo in right position
            {
                // flips intake upside down
                leftFlipServo.setPosition(leftFlipServo.getPosition() - distanceToFlip);
                rightFlipServo.setPosition(rightFlipServo.getPosition() + distanceToFlip);

                // spins until button not pressed (when driver has seen the pixel go into the depositor)
                while (gamepad1.y) {
                    // spins to eject pixels
                    leftIntakeServo.setPower(1);
                    rightIntakeServo.setPower(-1);
                }

                // stops spinning
                leftIntakeServo.setPower(0);
                rightIntakeServo.setPower(0);

                // flips intake back to regular
                leftFlipServo.setPosition(leftFlipServo.getPosition() + distanceToFlip);
                rightFlipServo.setPosition(rightFlipServo.getPosition() - distanceToFlip);
            }
        }
    }
}