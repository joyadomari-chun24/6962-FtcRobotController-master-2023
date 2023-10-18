/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This OpMode shows how to use Kauai Labs navX Micro Robotics Navigation
 * Sensor. It assumes that the sensor is configured with a name of "navx".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "navX test", group = "Sensor")
public class NavxManager {

    /** {@link IntegratingGyroscope} and it's base interface, {@link Gyroscope}) are common
     * interfaces supported by different gyro implementations. {@link NavxMicroNavigationSensor},
     * by contrast, provides functionality that is unique to the navX Micro sensor.
     *
     * //
     */
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetry;

    public NavxManager(Telemetry telemetry, HardwareMap hardwareMap) throws InterruptedException {
        this.telemetry = telemetry;
        // Get a reference to a Modern Robotics GyroSensor object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }

    public void navxTelemetry() throws InterruptedException
    {
        // Read dimensionalized data from the gyro. This gyro can report angular velocities
        // about all three axes. Additionally, it internally integrates the Z axis to
        // be able to report an absolute angular Z orientation.
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        //Intrinsic = navx remains constant zyx axis orientation to the field
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

        telemetry.addLine()
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.update();
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * @param degrees of the navx
     * @return the degrees mapped to a -1 to 1 float, which can be compared w/ joystick reading
     */
    float formatForJoystick(double degrees)
    {
        double inUpper = 180;
        double inLower = -180;
        int outUpper = -1;
        int outLower = 1;
        return (float)((degrees - inLower) * (outUpper - outLower) / (inUpper - inLower) + outUpper);
    }
}
