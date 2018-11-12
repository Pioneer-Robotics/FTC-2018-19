package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Motor channel:  Left  drive motor:        "motorLeft"
 * Motor channel:  Right drive motor:        "motorRight"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareLL5156
{
    /* Public OpMode members. */
    public DcMotor  motorLeft   = null;
    public DcMotor  motorRight  = null;
    public DcMotor  linearArm   = null;
    public Servo lunchBox   = null;
    public Servo rgtLatch   = null;
    public Servo lftLatch   = null;
    public DigitalChannel linearSwitch = null;

    //public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double lunchBoxMAX_POSITION = 0.875;
    public static final double lunchBoxMIN_POSITION = 0.55;
    public static final double rgtLatchMAX_POSITION = 0.95;
    public static final double rgtLatchMIN_POSITION = 0.67;
    public static final double lftLatchMAX_POSITION = 0.00;
    public static final double lftLatchMIN_POSITION = 0.28;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareLL5156(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        linearArm  = hwMap.get(DcMotor.class, "linearArm");
        linearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        linearSwitch = hwMap.get(DigitalChannel.class, "linearSwitch");

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        linearArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        lunchBox  = hwMap.get(Servo.class, "lunchBox");
        rgtLatch  = hwMap.get(Servo.class, "rgtLatch");
        lftLatch  = hwMap.get(Servo.class, "lftLatch");

        lunchBox.setPosition(lunchBoxMAX_POSITION);
        rgtLatch.setPosition(rgtLatchMAX_POSITION);
        lftLatch.setPosition(lftLatchMAX_POSITION);

    }
}