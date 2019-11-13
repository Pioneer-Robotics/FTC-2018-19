package org.firstinspires.ftc.teamcode.Input;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;

public class bMotor {

    //The ratio between how fast we are moving the motor and how much power we are using
    public double powerEncoderRatio;

    DcMotor motor;

    DeltaTime deltaTime;

    double encoderDelta;

    int lastEncoderReading;

    public bMotor(String motorName, OpMode op) {
        motor = op.hardwareMap.get(DcMotor.class, motorName);
    }

    public void Update() {
        deltaTime.Stop();
        encoderDelta = (motor.getCurrentPosition() - lastEncoderReading) / deltaTime.deltaTime();

        powerEncoderRatio = encoderDelta / motor.getPower();


        lastEncoderReading = motor.getCurrentPosition();
        deltaTime.Start();
    }
}
