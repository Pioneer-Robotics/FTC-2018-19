package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.sql.Time;

//A wrapper for DcMotors that has the ability to find the encoder to power ratio, used for real time trouble shooting. For more information see RobotDriveManager.java
public class bMotor {

    //The ratio between how fast we are moving the motor and how much power we are using
    public double powerEncoderRatio = 1;

    //This is set by the DriveManager and is responsible for ensuring that all power values don't exceed a motors capacity
    public double powerCoefficent = 1;

    public DcMotor motor;

    DeltaTime deltaTime = new DeltaTime();

    double encoderDelta = 0;

    int lastEncoderReading;

    int motorPosition;

    double assignedPower;

    OpMode opMode;

    String name;

    public bMotor(String motorName, OpMode op) {
        motor = op.hardwareMap.get(DcMotor.class, motorName);
        opMode = op;
        name = motorName;
    }

    public void Update(double targetMaxRatio) {
        deltaTime.Stop();

        //Fetches the motors real world position
        motorPosition = motor.getCurrentPosition();

        //Figure out how fast this wheel/motor is moving in encoder units per second
        encoderDelta = Math.abs(motorPosition - lastEncoderReading) / deltaTime.deltaTime();


//        opMode.telemetry.addData(name + " power", assignedPower);
//        opMode.telemetry.addData(name + " delta", encoderDelta);
//        opMode.telemetry.addData(name + " position", motorPosition);
//        opMode.telemetry.addData(name + " dt", deltaTime.deltaTime());
//        opMode.telemetry.addData(name + " ratio", powerEncoderRatio);


        //Avoid division by zero
        if (assignedPower != 0 && encoderDelta != 0) {

            //Sets up the ratio between the real speed and the applied power
            powerEncoderRatio = encoderDelta / assignedPower;

            //sets up the power coefficient that is used in all power assignment. Used to match the targetMaxRatio
            powerCoefficent = targetMaxRatio / powerEncoderRatio;

//            opMode.telemetry.addData(name, powerEncoderRatio);
        }

        //Set the new last encoder position to the current position for later use
        lastEncoderReading = motorPosition;

        deltaTime.Start();
    }

    //Set the power of the motor while taking account for the new max power
    public void setPower(double power) {
        assignedPower = power;

        //Set the corrected power
        motor.setPower(power * powerCoefficent);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }
}
