package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;

import java.util.HashSet;

//The idea here is that we have encoders that are able to set a speed but the encoder never gardeners that ALL wheels are able to move a the same speeds
//This class takes the weakest motor and sets other motors power relative to that one
public class RobotDriveManager {

    public double targetRatio;

    double lowestRatio;

    public bMotor frontLeft;

    public bMotor frontRight;

    public bMotor backLeft;

    public bMotor backRight;

    public RobotDriveManager(OpMode op, String fL, String fR, String bL, String bR) {
        frontLeft = new bMotor(fL, op);
        frontRight = new bMotor(fR, op);
        backLeft = new bMotor(bL, op);
        backRight = new bMotor(bR, op);

        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);
    }

    //An array of all of the above motors in that order
    public HashSet<bMotor> driveMotors = new HashSet<bMotor>();

    public void Update() {
        lowestRatio = 100000000;
        for (bMotor motor : driveMotors) {

            motor.Update(targetRatio);
            double ratio = motor.powerEncoderRatio;

            if (ratio < lowestRatio) {
                lowestRatio = ratio;
            }
        }

        targetRatio = lowestRatio;
    }
}
