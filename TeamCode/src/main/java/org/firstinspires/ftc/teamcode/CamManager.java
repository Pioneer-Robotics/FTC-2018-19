package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class CamManager extends Thread {
    private BNO055IMU imu;
    private Orientation angles;
    private HardwareLL5156 robot;
    boolean go;
    float reference;


    public void init(BNO055IMU imu, HardwareLL5156 robo_t) {
        this.imu = imu;
        this.robot = robo_t;

    }
    //a calibration of the imu needs to be put at the start of the Simple Auto
    /**NOTE by Jeremy 11/30/18: This should work, if the rest of Michael's code is good. What I did was set the calibration angle
    angleZero as the input in a final float format, so it can't change. We always input with angles.firstAngle.
    Then, angles.firstAngle is again repeatedly sensed, but since angleZero is a final value, it should be a reference point.
    Thus, the camera should move as intended, provided the camera movement code is correct.**/
    public void camVision(final float angleZero) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float angleDiff = ( angles.firstAngle - angleZero );

        if ((angles.firstAngle + angleDiff) >= (angleZero - 90)  && (angles.firstAngle + angleDiff) < angleZero ) {
            robot.Camera.setPosition((Math.abs(angles.firstAngle * (0.5/90)))   +    0.5);
            //robot is turned to the right, cam stays center with objects
        }

        if ((angles.firstAngle + angleDiff) > angleZero  && (angles.firstAngle + angleDiff) <= (angleZero + 90) ) {
            robot.Camera.setPosition((Math.abs(angles.firstAngle * (0.5/90)))   -    0.5);
            //robot is turned to the left, cam stays center with objects
        }
        else
        {
            robot.Camera.setPosition(0.5);
            //robot is past 90 degrees in either direction, cam moves to center
        }
    }
    public void run() {
        while (go) {
            camVision(reference);
        }
    }
}
