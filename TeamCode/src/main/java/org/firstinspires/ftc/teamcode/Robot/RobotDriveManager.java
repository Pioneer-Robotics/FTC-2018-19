package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;

import java.util.HashSet;

//The idea here is that we have encoders that are able to assign a specific speed to a motor. But the encoder never grantees that ALL motors are able to move at the same speeds (ei if one motors weaker than all of the others we need to adjust the stronger motors max speeds to work within the bounds of the weakest motor)
//This class takes the weakest motor and sets other motors power relative to that one
public class RobotDriveManager {

    ElapsedTime deltaTime = new ElapsedTime();

    ElapsedTime caibrationDeltaTime = new ElapsedTime();

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

    public void PreformInitalCalibration() {

        for (bMotor motor : driveMotors) {
            motor.setPower(1);
        }
        for (int i = 0; i < 100; i++) {

            UpdateCalibration();

            opMode.telemetry.addData("Current Target Ratio", targetRatio);
            opMode.telemetry.addData("Current Iteration ", i);

            for (bMotor motor : driveMotors) {
                motor.Calibrate(targetRatio);
            }

            opMode.telemetry.update();

            try {
                Thread.sleep((long) bMath.Clamp(100 - caibrationDeltaTime.milliseconds(), 0, 100));
            } catch (InterruptedException ex) {
                bTelemetry.Print("Wheel calibration failed: InterruptedException :(");
            }

            caibrationDeltaTime.reset();
        }

        for (bMotor motor : driveMotors) {
            motor.setPower(0);
        }

        bTelemetry.Print("Verifying");


        //If the the calibration is not valid rerun the calibration
        if (!VerifyCalibration()) {
            PreformInitalCalibration();
        }

    }


    public void UpdateCalibration() {

        lowestRatio = 100000000;
        for (bMotor motor : driveMotors) {

            double ratio = motor.powerEncoderRatio;

            if (ratio < lowestRatio) {
                lowestRatio = ratio;
            }
        }

        //Clamp the ratio between one rotation per second and the max rotation
        lowestRatio = bMath.Clamp(lowestRatio, RobotConfiguration.wheel_ticksPerRotation, RobotConfiguration.wheel_maxTicksPerSecond);

        //Update the target ratio
        targetRatio = lowestRatio;


        deltaTime.reset();
    }

    //Returns true if the wheel calibration is valid
    public boolean VerifyCalibration() {
        //The total combined speed coefficients of all of the motors, should be less than 4 at all times
        double totalCoefficient = 0;

        for (bMotor motor : driveMotors) {
            totalCoefficient += motor.powerCoefficent;
        }

        for (bMotor motor : driveMotors) {
            if (motor.powerCoefficent > 1) {
                return false;
            }
        }

        return totalCoefficient <= 4;
    }

    public void UpdateMotorDrive() {

    }
}
