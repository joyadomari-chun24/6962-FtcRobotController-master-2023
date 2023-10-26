package org.firstinspires.ftc.teamcode.SlideStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SpinMotor2 (Blocks to Java)")
public class SpinMotor2 extends LinearOpMode {

    private DcMotor Intake;
    private DcMotor Score;
    private Servo claw;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Score = hardwareMap.get(DcMotor.class, "Score");
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
//                Intake.setPower(gamepad1.left_stick_y);
                Score.setPower(gamepad2.right_stick_y);
                if (gamepad1.a) {
                    claw.setPosition(claw.getPosition() + 0.25);
                }
                if (gamepad1.b) {
                    claw.setPosition(claw.getPosition() - 0.25);
                }
            }
        }
    }
}
