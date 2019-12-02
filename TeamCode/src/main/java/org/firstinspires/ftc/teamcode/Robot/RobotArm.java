package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public double targetLengthSpeed;

    ElapsedTime deltaTime = new ElapsedTime();

    public RobotArm(LinearOpMode opMode, String armRotationMotor, String armSpoolMotor, String gripServo, String gripRotationServo) {
//            grip = opMode.hardwareMap.get(Servo.class, gripServo);
//            gripRotation = opMode.hardwareMap.get(Servo.class, gripRotationServo);
        Op = opMode;
        rotation = opMode.hardwareMap.get(DcMotor.class, armRotationMotor);
        length = opMode.hardwareMap.get(DcMotor.class, armSpoolMotor);

        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        length.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        length.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        length.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public double ThetaDegrees(Double k, Double H, double L, double d) {
        Double c = ((k * k) - (H * H) - (L * L) - (d * d)) / 2;
        Double x = (((d * c) - (H * Math.sqrt((((L * L) * (d * d)) + ((L * L) * (H * H))) - (c * c)))) / ((d * d) + (H * H))) + d;

        return Math.toDegrees(Math.atan((Math.sqrt((k * k) - (x * x)) - H) / (d - x)));
    }


    public void SetState(double targetAngle, double _targetLength, double angleSpeed, double lengthSpeed) {

        targetLengthSpeed = lengthSpeed;
        targetLength = _targetLength;
        rotation.setTargetPosition((int) ((double) -5679 * targetAngle));
//        length.setTargetPosition((int) ((double) -2623 * targetLength));

        rotation.setPower(angleSpeed);
        length.setPower(lengthSpeed / 10);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        length.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        length.setDirection(DcMotorSimple.Direction.REVERSE);
        deltaTime.reset();

        while (Op.opModeIsActive() && rotation.isBusy()) {
            Op.telemetry.addData("Length Power", length.getPower());
            Op.telemetry.addData("Length DT", deltaTime.seconds());


            Op.telemetry.update();

            deltaTime.reset();
        }

        rotation.setPower(0);
//            length.setPower(0);
    }

    public void PreformInitalCalibration() {

        //Face the arm all the way up and then test the speeds required to maintain a constant length
        SetState(1, 0.85, 1, 0);

        double encoderDelta = 1000;

        double lastPower = 0;

        int initialEncoderReading = 0;

        ElapsedTime calibrationDeltaTime = new ElapsedTime();

        calibrationDeltaTime.reset();

        for (int i = 0; i < 1000; i++) {

            //If the arm is moving then adjust then ramp up the power until it is not
            if (encoderDelta > 1) {
                lastPower = (double) i / (double) 1000;
                length.setPower(lastPower);

            }


            encoderDelta = (initialEncoderReading - length.getCurrentPosition()) / calibrationDeltaTime.seconds();

            initialEncoderReading = length.getCurrentPosition();

            calibrationDeltaTime.reset();
        }

    }
}
