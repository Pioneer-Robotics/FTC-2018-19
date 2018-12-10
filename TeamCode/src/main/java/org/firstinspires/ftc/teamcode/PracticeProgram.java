package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Hex;
import org.firstinspires.ftc.teamcode.HardwareInfinity;

@TeleOp
@Disabled
public class PracticeProgram extends LinearOpMode {
    HardwareInfinity robot = new HardwareInfinity();
    //private Gyroscope imu;
    private DcMotor motorRight;
    private DcMotor motorLeft;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    //private ColorSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        //imu = hardwareMap.get(Gyroscope.class, "imu");
        motorRight = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        //sensorColorRange = hardwareMap.get(ColorSensor.class, "sensorColorRange");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //set digital channel to input mode. Touch sensor
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            //motor run
            tgtPower = -this.gamepad1.left_stick_y;
            robot.motorRight.setPower(tgtPower);
            robot.motorLeft.setPower(tgtPower);
            //check to see if we need to move the servo.
            if(gamepad1.y){
                //move to 0 degrees.
                servoTest.setPosition(0);
            }
            else if(gamepad1.x || gamepad1.b){
                //move to 90 degrees.
                servoTest.setPosition(0.5);
            }
            else if(gamepad1.a){
                //move to 180 degrees.
                servoTest.setPosition(1);
            }

            //telemetry for servo
            telemetry.addData("Servo Position", servoTest.getPosition());
            //telemetry for motor
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorRight.getPower());

            //telemetry color sensor range
            //telemetry.addData("Distance (in)", sensorColorRange.());

            //telemetry color sensor color
            //telemetry.addData("Hue", "%x" , sensorColorRange.argb());
            //telemetry.addData("Colors", "red %d, green %d, blue %d", sensorColorRange.red(), sensorColorRange.green(), sensorColorRange.blue());
            //telemetry.addData("Light Intensity", sensorColorRange.alpha());
            //telemetry touch sensor. Is the button pressed?
            /*if(digitalTouch.getState() == false){
                //button is pressed.
                telemetry.addData("Button", "Pressed");
            } else {
                //button is not pressed.
                telemetry.addData("Button", "Not Pressed");
            }*/
            //telemetry base
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}