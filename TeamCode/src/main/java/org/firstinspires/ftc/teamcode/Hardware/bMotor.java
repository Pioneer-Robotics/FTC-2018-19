package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;

//A wrapper for DcMotors that has the ability to find an encoder to power ratio, used for real time trouble shooting
public class bMotor {

    //The ratio between how fast we are moving the motor and how much power we are using
    public double powerEncoderRatio = 1;

    //This is set by the DriveManager and is responsible for ensuring that all power values don't exceed a motors capacity
    public double powerCoefficent = 1;

    DcMotor motor;

    DeltaTime deltaTime = new DeltaTime();

    double encoderDelta = 1;

    int lastEncoderReading;

    OpMode opMode;

    String name;

    public bMotor(String motorName, OpMode op) {
        motor = op.hardwareMap.get(DcMotor.class, motorName);
        opMode = op;
        name = motorName;
    }

    public void Update(double targetMaxRatio) {
        deltaTime.Stop();

        //Figure out how fast this wheel/motor is moving in encoder units per second
        encoderDelta = (motor.getCurrentPosition() - lastEncoderReading) / deltaTime.deltaTime();

        if (Math.abs(motor.getPower()) > 0 && deltaTime.deltaTime() > 0 && encoderDelta != 0) {

            //Set the ratio
            powerEncoderRatio = encoderDelta / motor.getPower();

            powerCoefficent = targetMaxRatio / powerEncoderRatio;

            opMode.telemetry.addData(name, powerEncoderRatio);

            //Set the new last encoder position to the current position for later use
            lastEncoderReading = motor.getCurrentPosition();
        }

        deltaTime.Start();
    }

    public void setPower(double power) {
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
