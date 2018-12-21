package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

public class CamManager extends Thread {
    private BNO055IMU imu;
    private HardwareInfinity robot;
    private CVManager CamCV = new CVManager();
    boolean go = true;
    float reference;
    int mode = 0;
    double camSpeed = 0.001;
    private static boolean canTrack = true; //set this to false if the app crashes with tracking
    private float screenX= 800; //pixel size of tflow
    DecimalFormat df = new DecimalFormat("#.###");


    void init(BNO055IMU imu, HardwareInfinity robo_t, HardwareMap hw, CVManager tf) {
        this.imu = imu;
        this.robot = robo_t;
        this.CamCV = tf;
        this.CamCV.disable = false;
        //if (!this.CamCV.isAlive() && canTrack) this.CamCV.start();

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
            camSpeed = -camSpeed;
        }
        if (robot.Camera.getPosition()+ camSpeed >=1) {
            robot.Camera.setPosition(1);
        } else if (robot.Camera.getPosition()+ camSpeed <=0.5) {
            robot.Camera.setPosition(0.5);
        } else {
            robot.Camera.setPosition(robot.Camera.getPosition()+ camSpeed);
        }
    }
    private void track(double goldX) {
      //0 & 0 =left
        //1 & 780=right
        /*double pixelDiff = robot.Camera.getPosition() + 0.0025*(goldX-(screenX/2));
        if (goldX != screenX/2 && robot.Camera.getPosition()<= 1 && robot.Camera.getPosition()>= 0 && pixelDiff <=1 && pixelDiff >=0){
            robot.Camera.setPosition(pixelDiff);
        }
        else if (pixelDiff >1){
            robot.Camera.setPosition(1);
        }
        else if (pixelDiff < 0){
            robot.Camera.setPosition(0);
        }*/
        if (goldX>((screenX/2)+70)) {
            camSpeed = 0.001;
            /*try {
                Thread.sleep(10);
            } catch (InterruptedException e){
                int Jwee;
            }*/
        } else if (goldX<((screenX/2)-70)) {
            camSpeed = -0.001;
            /*try {
                Thread.sleep(10);
            } catch (InterruptedException e){
                int Jwee;
            }*/
        } else camSpeed = 0;
        if (robot.Camera.getPosition()+ camSpeed >=1) {
            robot.Camera.setPosition(1);
        } else if (robot.Camera.getPosition()+ camSpeed <=0) {
            robot.Camera.setPosition(0);
        } else {
            robot.Camera.setPosition(robot.Camera.getPosition()+ camSpeed);
        }
    }
    public void run() {
        while (go) {
            if (mode == 0) {
                camVision(reference);
            } else if (mode == 1) {
                if (camSpeed == 0) {
                    camSpeed = 0.001;
                }
                scan();
            } else if (mode == 2 && canTrack) {
                CamCV.track = true;
                track(CamCV.mineralX);
            }
            if (mode != 2) {
                CamCV.track = false;
            }
            if (CamCV.mineralX == 0 && mode == 2) {
                mode = 1;
            } else if (mode == 1 && CamCV.mineralX != 0) {
                mode = 2;
            }
        }
        CamCV.go = false;
    }
}
