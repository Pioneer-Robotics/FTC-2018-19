package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

public class CamManager extends Thread {
    //initialize al of the necessary variables
    private BNO055IMU imu;
    private HardwareInfinity robot;
    private CVManager CamCV;
    boolean go = true;
    float reference;
    int mode = 0;
    private double camSpeed = 0.002;
    float lBound;
    float rBound;
    private DecimalFormat df = new DecimalFormat("#.###");


    public void init(HardwareInfinity arobot, CVManager tf) {
        //pass necessary classes and sensors from the main hardware class
        this.imu = arobot.imu;
        this.robot = arobot;
        this.CamCV = tf;
        //this.CamCV.autoDisable = false;
        //if (!this.CamCV.isAlive() && canTrack) this.CamCV.start();

    }

    private void camVision(final float angleZero) {
        //set the camera servo to compList value relative to the IMU
        //get IMU value
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //find angle delta
        float angleDiff = ( angles.firstAngle - angleZero );
        //check if the servo can be moved to the right location and if so then do it
        if (((angles.firstAngle + angleDiff) >= angleZero - 150)  && ((angles.firstAngle + angleDiff) <= (angleZero + 150)) ) {
            robot.Camera.setPosition(1-Math.abs( Float.parseFloat(df.format(angles.firstAngle)) * (0.5/90) - 0.55));
            //cam stays center while robot turns
        }
    }
    private void scan(float lBound,float rBound) {
        //move the camera back and forth to look for the gold
        if (robot.Camera.getPosition()>= rBound || robot.Camera.getPosition() <= lBound) {
            camSpeed = -camSpeed;
        }
        if (robot.Camera.getPosition()+ camSpeed >=rBound) {
            robot.Camera.setPosition(rBound);
        } else if (robot.Camera.getPosition()+ camSpeed <=lBound) {
            robot.Camera.setPosition(lBound);
        } else {
            robot.Camera.setPosition(robot.Camera.getPosition()+ camSpeed);
        }
    }
    private void retur(final float angleZero) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //find angle delta
        float angleDiff = ( angles.firstAngle - angleZero );
        if (((angles.firstAngle + angleDiff) >= angleZero - 150)  && ((angles.firstAngle + angleDiff) <= (angleZero + 150)) && robot.Camera.getPosition() >= 1-Math.abs( Float.parseFloat(df.format(angles.firstAngle)) * (0.5/90) - 0.55)) {
            camSpeed = -Math.abs(camSpeed);
        } else if (((angles.firstAngle + angleDiff) >= angleZero - 150)  && ((angles.firstAngle + angleDiff) <= (angleZero + 150))) {
            camSpeed = Math.abs(camSpeed);
        }
        robot.Camera.setPosition(robot.Camera.getPosition()+ camSpeed);
    }
    private void track(double goldX) {
        // move to the position of the gold
        //0 & 0 =left
        //1 & 780=right
        //pixel size of tflow
        float screenX = 800;
        if (goldX>((screenX /2)+70)) {
            camSpeed = 0.001;
        } else if (goldX<((screenX /2)-70)) {
            camSpeed = -0.001;
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
        //main loop of the Camera Manager
        //check if we can run
        while (go) {
            //take different actions based on the mode
            //set this to false if the app crashes with tracking
            if (mode == 0) {
                // counteract robot turning
                camVision(reference);
            } else if (mode == 1) {
                //scan in mode one
                if (camSpeed == 0) {
                    camSpeed = 0.0008;
                }
                scan(lBound,rBound);
            } else if (mode == 2) {
                //track the gold in mode 2
                if (!CamCV.isAlive()) {
                    CamCV.go = true;
                    CamCV.start();
                }
                CamCV.track = true;
                track(CamCV.minDat[0]);
            } else if (mode == 3) {
                retur(reference);
            }
            //Manage auto changing between the modes
            if (mode != 2) {
                CamCV.track = false;
            }
            if (CamCV.status == 3 && mode == 2) {
                mode = 1;
            } else if (mode == 1 && CamCV.status != 3) {
                mode = 2;
            }
        }
        CamCV.go = false;
    }
}
