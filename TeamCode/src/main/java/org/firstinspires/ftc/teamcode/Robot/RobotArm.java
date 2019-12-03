package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.bDataManager;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

import static java.lang.Thread.sleep;

public class RobotArm extends Thread {

    LinearOpMode Op;

    //Arm height motor
    public DcMotor rotation;

    //Controls arm length (spool)
    public DcMotor length;

    public Servo gripRotation;
    public Servo grip;

    public double targetLength;
    public double currentLengthSpeed;
    public double targetLengthSpeed;

    ElapsedTime deltaTime = new ElapsedTime();

    public RobotArm(LinearOpMode opMode, String armRotationMotor, String armSpoolMotor, String gripServo, String gripRotationServo) {
        grip = opMode.hardwareMap.get(Servo.class, gripServo);
        gripRotation = opMode.hardwareMap.get(Servo.class, gripRotationServo);
        Op = opMode;
        rotation = opMode.hardwareMap.get(DcMotor.class, armRotationMotor);
        length = opMode.hardwareMap.get(DcMotor.class, armSpoolMotor);

        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        length.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        length.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        start();
    }


    public double ThetaDegrees(Double k, Double H, double L, double d) {
        Double c = ((k * k) - (H * H) - (L * L) - (d * d)) / 2;
        Double x = (((d * c) - (H * Math.sqrt((((L * L) * (d * d)) + ((L * L) * (H * H))) - (c * c)))) / ((d * d) + (H * H))) + d;

        return Math.toDegrees(Math.atan((Math.sqrt((k * k) - (x * x)) - H) / (d - x)));
    }


    public void SetArmState(double targetAngle, double _targetLength, double angleSpeed, double _lengthSpeed) {

        targetLengthSpeed = _lengthSpeed;
        targetLength = _targetLength;

        rotation.setTargetPosition((int) ((double) -5679 * targetAngle));
        length.setTargetPosition((int) ((double) -2623 * targetLength));

        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentLengthSpeed = 0;


        while (Op.opModeIsActive() && rotation.isBusy()) {
            Op.telemetry.addData("Length Power", length.getPower());
            Op.telemetry.addData("Length DT", deltaTime.seconds());


            Op.telemetry.update();
        }

        rotation.setPower(0);
    }

    public void SetGripState(double strength, double rotation) {
        grip.setPosition(strength);
        gripRotation.setPosition(rotation);
    }

    public void run() {
        currentLengthSpeed = bMath.MoveTowards(currentLengthSpeed, targetLengthSpeed, deltaTime.seconds() * 0.5);

        length.setPower(currentLengthSpeed);
        length.setTargetPosition((int) ((double) -2623 * targetLength));
    }

}
