package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.text.DecimalFormat;

public class CamManager extends Thread {
    private BNO055IMU imu;
    private HardwareInfinity robot;
    private CVManager CamCV = new CVManager();
    boolean go = true;
    float reference;
    int mode = 0;
    double dir = 0.003;
    private float screenX= 760; //pixel size of tflow
    DecimalFormat df = new DecimalFormat("#.###");


    void init(BNO055IMU imu, HardwareInfinity robo_t, HardwareMap hw, VuforiaLocalizer vu) {
        this.imu = imu;
        this.robot = robo_t;
        CamCV.initwoVu(vu ,hw.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hw.appContext.getPackageName()));
        CamCV.mode = 2;

    }

    private void camVision(final float angleZero) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float angleDiff = ( angles.firstAngle - angleZero );

        if (((angles.firstAngle + angleDiff) >= angleZero - 150)  && ((angles.firstAngle + angleDiff) <= (angleZero + 150)) ) {
            robot.Camera.setPosition(1-Math.abs( Float.parseFloat(df.format(angles.firstAngle)) * (0.5/90) - 0.5));
            //cam stays center while robot turns
        }
    }
    private void scan() {
        if (robot.Camera.getPosition()>= 1 || robot.Camera.getPosition() <= 0.5) {
            dir = -dir;
        }
        if (robot.Camera.getPosition()+dir>=1) {
            robot.Camera.setPosition(1);
        } else if (robot.Camera.getPosition()+dir<=0.5) {
            robot.Camera.setPosition(0.5);
        } else {
            robot.Camera.setPosition(robot.Camera.getPosition()+dir);
        }
    }
    private void track(double goldX) {
      //0 & 0 =left
        //1 & 780=right
        double pixelDiff = robot.Camera.getPosition() + 0.002631579*(goldX-380);
        if (goldX != screenX/2 && robot.Camera.getPosition()<= 1 && robot.Camera.getPosition()>= 0 && pixelDiff <=1 && pixelDiff >=0){
            robot.Camera.setPosition(pixelDiff);
        }
        else if (pixelDiff >1){
            robot.Camera.setPosition(1);
        }
        else if (pixelDiff < 0){
            robot.Camera.setPosition(0);
        }





    }
    public void run() {
        while (go) {
            if (mode == 0) {
                camVision(reference);
            } else if (mode == 1) {
                scan();
            } else if (mode == 2) {
                if (!CamCV.isAlive()) {
                    CamCV.start();
                }
                track(CamCV.mineralX);
            }
        }
    }
}
