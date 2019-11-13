package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;

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

        driveMotors[0] = frontLeft;
        driveMotors[1] = frontRight;
        driveMotors[2] = backLeft;
        driveMotors[3] = backRight;
    }

    //An array of all of the above motors in that order
    public bMotor[] driveMotors = new bMotor[4];

    public void Update() {
        lowestRatio = 100000000;
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].Update(targetRatio);
            double ratio = driveMotors[i].powerEncoderRatio;

            if (ratio < lowestRatio) {
                lowestRatio = ratio;
            }
        }
        targetRatio = lowestRatio;
    }
}
