package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.text.DecimalFormat;

public class CamManager extends Thread {
    private BNO055IMU imu;
    private HardwareInfinity robot;
    boolean go = true;
    float reference;
    DecimalFormat df = new DecimalFormat("#.###");


    void init(BNO055IMU imu, HardwareInfinity robo_t) {
        this.imu = imu;
        this.robot = robo_t;

    }

    private void camVision(final float angleZero) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float angleDiff = ( angles.firstAngle - angleZero );

        if (((angles.firstAngle + angleDiff) >= angleZero - 150)  && ((angles.firstAngle + angleDiff) <= (angleZero + 150)) ) {
            robot.Camera.setPosition(0.95-Math.abs( Float.parseFloat(df.format(angles.firstAngle)) * (0.5/90) - 0.5));
            //cam stays center while robot turns
        }
    }


    public void run() {
        while (go) {
            camVision(reference);
        }
    }
}
