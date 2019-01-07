package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Motor channel:  Left  drive motor:        "motorLeft"
 * Motor channel:  Right drive motor:        "motorRight"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareInfinity
{
    /* Public OpMode members. */
    DcMotor  motorLeft;
    DcMotor  motorRight;
    DcMotor  linearArm;
    DcMotor armBase;
    DcMotor FBar;
    DcMotor Succq;
    Servo lunchBox;
    Servo Latch;
    Servo Camera;
    Servo dropTop;
    DigitalChannel botSwitch;
    DigitalChannel topSwitch;
    TouchSensor trigger;
    BNO055IMU imu;
    BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();

    //public Servo    rightClaw   = null;

    static final double lunchBoxMAX_POSITION = 0.05;
    static final double lunchBoxMIN_POSITION = 0.7;
    static final double LatchMAX_POSITION = 0;
    static final double LatchMIN_POSITION = 1;
    static final double DT_MIN = 0.55;
    static final double DT_MAX = 0.9;

    /* local OpMode members. */
    HardwareMap hwMap;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareInfinity(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        linearArm  = hwMap.get(DcMotor.class, "linearArm");
        armBase = hwMap.get(DcMotor.class, "armBase");
        FBar = hwMap.get(DcMotor.class, "fBar");
        Succq = hwMap.get(DcMotor.class, "succq");
        linearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        FBar.setDirection(DcMotor.Direction.REVERSE);
        botSwitch = hwMap.get(DigitalChannel.class, "botSwitch");
        topSwitch = hwMap.get(DigitalChannel.class, "topSwitch");
        trigger = hwMap.get(TouchSensor.class, "trigger");
        topSwitch.setMode(DigitalChannel.Mode.INPUT);
        botSwitch.setMode(DigitalChannel.Mode.INPUT);
        imu = hwMap.get(BNO055IMU.class, "imu");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        imu.initialize(IParameters);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        linearArm.setPower(0);
        armBase.setPower(0);
        FBar.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Succq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        lunchBox  = hwMap.get(Servo.class, "lunchBox");
        Latch  = hwMap.get(Servo.class, "Latch");
        Camera = hwMap.get(Servo.class, "Camera");
        dropTop = hwMap.get(Servo.class, "dropTop");

        lunchBox.setPosition(lunchBoxMAX_POSITION);
        Latch.setPosition(LatchMAX_POSITION);
        Camera.setPosition(0.55);
        dropTop.setPosition(DT_MAX);

    }
}