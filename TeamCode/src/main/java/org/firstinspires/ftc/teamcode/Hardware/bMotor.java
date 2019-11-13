package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;

//A wrapper for DcMotors that has the ability to find an encoder to power ratio, used for real time trouble shooting
public class bMotor {

    //The ratio between how fast we are moving the motor and how much power we are using
    public double powerEncoderRatio;

    //This is set by the DriveManager and is responsible for ensuring that all power values don't exceed a motors capacity
    public double powerCoefficent;

    DcMotor motor;

    DeltaTime deltaTime;

    double encoderDelta;

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

        //Set the ratio
        powerEncoderRatio = encoderDelta / motor.getPower();

        powerCoefficent = targetMaxRatio / powerEncoderRatio;

        opMode.telemetry.addData(name, powerEncoderRatio);

        //Set the new last encoder position to the current position for later use
        lastEncoderReading = motor.getCurrentPosition();
        deltaTime.Start();
    }

    public void SetPower(double power) {
        motor.setPower(power * powerCoefficent);
    }

    public void SetMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void SetDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public void SetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }
}
