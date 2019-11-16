package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.HashSet;

//The idea here is that we have encoders that are able to set a speed but the encoder never gardeners that ALL wheels are able to move a the same speeds
//This class takes the weakest motor and sets other motors power relative to that one
public class RobotDriveManager {

    DeltaTime deltaTime = new DeltaTime();

    public double targetRatio = 2;

    double lowestRatio;

    public bMotor frontLeft;

    public bMotor frontRight;

    public bMotor backLeft;

    public bMotor backRight;

    public OpMode opMode;

    public RobotDriveManager(OpMode op, String fL, String fR, String bL, String bR) {
        frontLeft = new bMotor(fL, op);
        frontRight = new bMotor(fR, op);
        backLeft = new bMotor(bL, op);
        backRight = new bMotor(bR, op);

        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);

        opMode = op;
    }

    //An array of all of the above motors in that order
    public HashSet<bMotor> driveMotors = new HashSet<bMotor>();

    public void Update() {
        deltaTime.Stop();
        lowestRatio = 100000000;
        for (bMotor motor : driveMotors) {

            motor.Update(targetRatio);
            double ratio = motor.powerEncoderRatio;

            if (ratio < lowestRatio) {
                lowestRatio = ratio;
            }
        }
        //Lock the ratio, we don't want the robot to set the max speed to 0
        lowestRatio = bMath.Clamp(lowestRatio, 0.5, 3);

        //Lerp the target ratio ratio
        targetRatio = bMath.Lerp(targetRatio, lowestRatio, deltaTime.deltaTime() * 0.5);

        opMode.telemetry.addData("Lowest Ratio Target", lowestRatio);
        opMode.telemetry.addData("Current Ratio Target", targetRatio);


        deltaTime.Start();
    }
}
