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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * {@link BNO055IMUDriverTesting} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "BNO055 IMU Driver Test", group = "Sensor")
@Disabled // Comment this out to add to the opmode list
public class BNO055IMUDriverTesting extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    // State used for updating telemetry
    Orientation angles;
    Orientation angles1;
    Orientation oldAng;
    Orientation oldAng1;
    //Acceleration gravity;
    private HardwareInfinity robot = new HardwareInfinity();
    ElapsedTime runtime = new ElapsedTime();
    double time;
    double maxt;
    double maxd;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {
        robot.init(hardwareMap);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        oldAng = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //BNO055IMU.Register reg = new BNO055IMU.Register();
        //reg.bVal = (byte)0xA0;
        //private SensorManager mSensorManager;
        //private Sensor mSensor;

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            oldAng = angles;
            oldAng1 = angles1;
            runtime.reset();
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles1   = robot.imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            time = runtime.milliseconds();
            if (time>maxt) maxt = time;
            if (angles.firstAngle - oldAng.firstAngle>maxt) maxd = angles.firstAngle - oldAng.firstAngle;
            telemetry.update();
            telemetry.addLine()
                    .addData("heading", "%.5f", angles.thirdAngle)
                    .addData("roll", "%.5f", angles.thirdAngle)
                    .addData("pitch","%.5f", angles.thirdAngle);
            /*telemetry.addLine()
                    .addData("nheading", "%.5f", imu.read())
                    .addData("nroll", "%.5f", angles.thirdAngle)
                    .addData("npitch","%.5f", angles.thirdAngle);*/
            telemetry.addLine()
                    .addData("Latency: ", "%.8f", time);
            telemetry.addLine()
                    .addData("Delta H: ", "%.8f", angles.firstAngle - oldAng.firstAngle)
                    .addData("Delta R: ", "%.8f", angles.secondAngle - oldAng.secondAngle)
                    .addData("Delta P: ", "%.8f", angles.thirdAngle - oldAng.thirdAngle);
            telemetry.addLine()
                    .addData("Max Latency: ", "%.2f", maxt)
                    .addData("Max Delta: ", "%.2f", maxd);
            telemetry.addLine()
                    .addData("heading1", "%.5f", angles1.thirdAngle)
                    .addData("roll1", "%.5f", angles1.thirdAngle)
                    .addData("pitch1","%.5f", angles1.thirdAngle);
            /*telemetry.addLine()
                    .addData("nheading", "%.5f", imu.read())
                    .addData("nroll", "%.5f", angles.thirdAngle)
                    .addData("npitch","%.5f", angles.thirdAngle);*/
            telemetry.addLine()
                    .addData("Delta H1: ", "%.8f", angles1.firstAngle - oldAng1.firstAngle)
                    .addData("Delta R1: ", "%.8f", angles1.secondAngle - oldAng1.secondAngle)
                    .addData("Delta P1: ", "%.8f", angles1.thirdAngle - oldAng1.thirdAngle);
            telemetry.addData("Drift:", ".10f", angles.firstAngle-angles1.firstAngle);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                oldAng = angles;
                runtime.reset();
                angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angles1   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                time = runtime.milliseconds();
                //gravity  = imu.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return robot.imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", "%.5f", angles.thirdAngle)
            .addData("roll", "%.5f", angles.thirdAngle)
            .addData("pitch","%.5f", angles.thirdAngle);
        telemetry.addLine()
                .addData("Latency: ", "%.8f", time);
        telemetry.addLine()
                .addData("Delta H: ", "%.8f", angles.firstAngle - oldAng.firstAngle)
                .addData("Delta R: ", "%.8f", angles.secondAngle - oldAng.secondAngle)
                .addData("Delta P: ", "%.8f", angles.thirdAngle - oldAng.thirdAngle);

        /*telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                }); */
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
