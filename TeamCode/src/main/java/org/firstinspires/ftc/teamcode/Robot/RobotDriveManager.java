package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.HashSet;

//The idea here is that we have encoders that are able to assign a specific speed to a motor. But the encoder never grantees that ALL motors are able to move at the same speeds (ei if one motors weaker than all of the others we need to adjust the stronger motors max speeds to work within the bounds of the weakest motor)
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

    //An array of all of the above motors
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

        //Clamp the ratio between one rotation per second and the max rotation
        lowestRatio = bMath.Clamp(lowestRatio, RobotConfiguration.wheel_ticksPerRotation, RobotConfiguration.wheel_maxTicksPerSecond);

        //Update the target ratio
        targetRatio = lowestRatio;

        //Update all motors with the latest ratio, this might not be needed but should prevent off by one cycle issues
        for (bMotor motor : driveMotors) {
            motor.Update(targetRatio);
        }

        opMode.telemetry.addData("Lowest Ratio Target", lowestRatio);
        opMode.telemetry.addData("Current Ratio Target", targetRatio);

        deltaTime.Start();
    }
}
