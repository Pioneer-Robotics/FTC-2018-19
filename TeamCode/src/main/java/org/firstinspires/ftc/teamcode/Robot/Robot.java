package org.firstinspires.ftc.teamcode.Robot;

import android.content.Context;
import android.renderscript.Double2;
import android.renderscript.Double4;

import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bDataManger;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Hardware.bIMU;
import org.firstinspires.ftc.teamcode.R;

import java.util.concurrent.atomic.AtomicBoolean;

import javax.crypto.spec.OAEPParameterSpec;

//TODO: clean up the canmove system
public class Robot extends Thread {

    //Static instance. Only have one robot at a time and access it from here (THERE CAN BE ONLY ONE)
    public static Robot instance;

    public RobotArm arm;

    //Our wee little wall tracker
    public RobotWallTrack wallTrack = new RobotWallTrack();

    //The current IMU rotation, threaded
    double rotation;

    public bIMU imu = new bIMU();

    //Hardware!
    public RobotDriveManager driveManager;

    double threadTimer;

    public ElapsedTime threadDeltaTime = new ElapsedTime();

    public bDataManger dataManger = new bDataManger();


    public LinearOpMode Op;
//    public OpMode LinearOpMode;

    //If our thread is running, using atomics to avoid thread conflicts. Might not be completely necessary
    AtomicBoolean threadRunning = new AtomicBoolean();

    public SensorREVColorDistance colorDistanceFront;

    public void init(HardwareMap hardwareMap, LinearOpMode opmode) {

        //Start the printer
        bTelemetry.Start(opmode);

        //Set up the instance (safety checks might be a good idea at some point)
        instance = this;
        bTelemetry.Print("Robot instance assigned.");

        //Set the opmode
        Op = opmode;

        //Find the motors
        driveManager = new RobotDriveManager(opmode, RobotConfiguration.wheel_frontLeft, RobotConfiguration.wheel_frontRight, RobotConfiguration.wheel_backLeft, RobotConfiguration.wheel_backRight);

//        frontLeft = hardwareMap.get(DcMotor.class, RobotConfiguration.wheel_frontLeft);
//        frontRight = hardwareMap.get(DcMotor.class, RobotConfiguration.wheel_frontRight);
//        backLeft = hardwareMap.get(DcMotor.class, RobotConfiguration.wheel_backLeft);
//        backRight = hardwareMap.get(DcMotor.class, RobotConfiguration.wheel_backRight);
        bTelemetry.Print("Robot wheels assigned.");
        bTelemetry.Print("Robot motors configured in the DriveManager.");

        arm = new RobotArm(opmode, RobotConfiguration.arm_rotationMotor, RobotConfiguration.arm_lengthMotor, RobotConfiguration.arm_gripServo, RobotConfiguration.arm_gripRotationServo);

//        gripServo = hardwareMap.get(Servo.class, "Grip");
//        armWintch = hardwareMap.get(DcMotor.class, "Arm");

        //Left wheels are reversed so power 1,1,1,1 moves us forward
        driveManager.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveManager.backLeft.setDirection(DcMotor.Direction.REVERSE);


        //Init the motors for use. NTS: If you don't do this the robot does not like to move with math
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bTelemetry.Print("Wheel encoders initialized.");


        //Set up the IMU(s)
        imu.Start(opmode, RobotConfiguration.imu_0, RobotConfiguration.imu_1);
        bTelemetry.Print("IMU's initialized.");

        //Set up the wall tracker, this uses ALL the lasers so make sure they all work before running this
        wallTrack.Start(opmode);
        bTelemetry.Print("Walltracker initialized.");

        //Starts the 'run' thread
        start();
        bTelemetry.Print("Robot thread initialized.");

        bTelemetry.Print("Robot start up successful. Preparing to read wheel calibration data...");

        dataManger.Start();

        bTelemetry.Print("bDataManager started.");


        driveManager.frontLeft.powerCoefficent = dataManger.readData("wheel_front_left_powerCo", -1);
        bTelemetry.Print("      Front Left  : " + driveManager.frontLeft.powerCoefficent);
        driveManager.frontRight.powerCoefficent = dataManger.readData("wheel_front_right_powerCo", -1);
        bTelemetry.Print("      Front Right : " + driveManager.frontRight.powerCoefficent);
        driveManager.backLeft.powerCoefficent = dataManger.readData("wheel_back_left_powerCo", -1);
        bTelemetry.Print("      Back Left   : " + driveManager.backLeft.powerCoefficent);
        driveManager.backRight.powerCoefficent = dataManger.readData("wheel_back_right_powerCo", -1);
        bTelemetry.Print("      Back Right  : " + driveManager.backRight.powerCoefficent);

        bTelemetry.Print("Wheel boot successful. Ready to operate!");

    }


    //A fancy version of init used for calibrating the robot, not to be used in any offical match as calibration will take anywhere from 10 to 30 seconds
    public void initCalibration(HardwareMap hardwareMap, LinearOpMode opmode) {

        //Start the printer
        bTelemetry.Start(opmode);

        //Set up the instance (safety checks might be a good idea at some point)
        instance = this;
        bTelemetry.Print("Robot instance assigned.");

        //Set the opmode
        Op = opmode;

        //Find the motors
        driveManager = new RobotDriveManager(opmode, RobotConfiguration.wheel_frontLeft, RobotConfiguration.wheel_frontRight, RobotConfiguration.wheel_backLeft, RobotConfiguration.wheel_backRight);

        bTelemetry.Print("Robot wheels assigned.");
        bTelemetry.Print("Robot motors configured in the DriveManager.");

        //Left wheels are reversed so power 1,1,1,1 moves us forward
        driveManager.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveManager.backLeft.setDirection(DcMotor.Direction.REVERSE);


        //Init the motors for use.
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bTelemetry.Print("Wheel encoders initialized.");


        //Set up the IMU(s)
        imu.Start(opmode, RobotConfiguration.imu_0, RobotConfiguration.imu_1);
        bTelemetry.Print("IMU's initialized.");

        //Set up the wall tracker, this uses ALL the lasers so make sure they all work before running this
        wallTrack.Start(opmode);
        bTelemetry.Print("Walltracker initialized.");

        //Starts the 'run' thread
        start();
        bTelemetry.Print("Robot thread initialized.");

        bTelemetry.Print("Robot start up successful. Preparing for initial wheel calibration!");

        dataManger.Start();

        bTelemetry.Print("bDataManager started.");

        bTelemetry.Print("Robot start up successful. Running initial wheel calibration...");

        driveManager.PreformInitalCalibration();

        bTelemetry.Print("Wheel boot successful. Writing results...");

        dataManger.writeData("wheel_front_left_powerCo", driveManager.frontLeft.powerCoefficent);
        dataManger.writeData("wheel_front_right_powerCo", driveManager.frontRight.powerCoefficent);
        dataManger.writeData("wheel_back_left_powerCo", driveManager.backLeft.powerCoefficent);
        dataManger.writeData("wheel_back_right_powerCo", driveManager.backRight.powerCoefficent);

        bTelemetry.Print("Wheel write successful.");

        bTelemetry.Print("Calibration complete, pleasure doing business with you.");


    }

    //Threaded run method, right now this is just for IMU stuff, at some point we might put some avoidance stuff in here (background wall tracking?) (average out 2IMU's for extra strain of the thread?)
    public void run() {
        threadRunning.set(true);

        while (threadRunning.get()) {

            //Update our 'rotation' value
            BackgroundRotation();

//            threadTimer += threadDeltaTime.seconds();
//            Op.telemetry.update();

            if (!Op.gamepad1.b) {
                arm.length.setPower(bMath.MoveTowards(arm.length.getPower(), arm.targetLengthSpeed, arm.deltaTime.seconds() * 0.5));
                arm.length.setTargetPosition((int) ((double) -2623 * arm.targetLength));
            }
            threadDeltaTime.reset();
        }


    }


    public void BackgroundRotation() {
        //Updates the current rotation
        rotation = imu.getRotation(AngleUnit.DEGREES);
    }


    public void Stop() {
        threadRunning.set(false);
        SetPowerDouble4(0, 0, 0, 0, 0);
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //<editor-fold desc="Movement">

    /**
     * Uses
     *
     * @param movementAngle The angle  that we want to move along, try to keep its magnitude under 180
     * @param movementSpeed How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void MoveSimple(double movementAngle, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementAngle);
        SetPowerDouble4(v, movementSpeed);
    }

    /**
     * Uses
     *
     * @param movementVector The vector that we want to move along
     * @param movementSpeed  How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void MoveSimple(Double2 movementVector, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementVector);
        SetPowerDouble4(v, movementSpeed);
    }

    /**
     * Uses
     *
     * @param movementAngle The angle  that we want to move along, try to keep its magnitude under 180
     * @param movementSpeed How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void MoveSimple(double movementAngle, double movementSpeed, double rotationPower) {
        Double4 v = bMath.getMecMovementSimple(movementAngle, rotationPower);
        SetPowerDouble4(v, movementSpeed);
    }

    /**
     * Uses
     *
     * @param movementVector The vector that we want to move along
     * @param movementSpeed  How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void MoveSimple(Double2 movementVector, double movementSpeed, double rotationPower) {
        Double4 v = bMath.getMecMovementSimple(movementVector, rotationPower);
        SetPowerDouble4(v, movementSpeed);
    }


    /**
     * @param movementAngle The angle (relative to the phoneside of the bot) that we want to move along, try to keep its magnitude under 180
     * @param movementSpeed How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     * @param angle         The angle that we want the robot to rotate too. It's actually witchcraft and might need some more research/testing
     */
    public void MoveComplex(double movementAngle, double movementSpeed, double angle) {
        Double4 v = bMath.getMecMovement(movementAngle, angle);
        SetPowerDouble4(v, movementSpeed);
    }

    /**
     * @param movementVector The vector (relative to the phoneside of the bot) that we want to move along
     * @param movementSpeed  How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     * @param angle          The angle that we want the robot to rotate too. It's actually witchcraft and might need some more research/testing
     */
    public void MoveComplex(Double2 movementVector, double movementSpeed, double angle) {
        Double4 v = bMath.getMecMovement(movementVector, angle);
        SetPowerDouble4(v, movementSpeed);
    }

    public void RotateSimple(double rotationSpeed) {
        Double4 v = bMath.getRotationSimple(rotationSpeed);
        SetPowerDouble4(v, 1);
    }

    PID rotationPID_test = new PID();

    //
    public void RotatePID(double angle, double rotationSpeed, int cycles) {

        //P of 3 and 0 for other gains seems to work really well
//        rotationPID_test.Start(3, 0, 0.1);

        rotationPID_test.Start(4.02, 0.0032, 0.0876);
//        rotationPID_test.Start(4.01, 0.003, 0.0876);

//        rotationPID_test.Start(1, 0.075, 0.022);

//        rotationPID_test.Start(3, 0.21, 0.69);
//        rotationPID_test.Start(0.5, 0.075, 0.015);
//        rotationPID_test.Start(1, 0.25, 0.035);
//        rotationPID_test.Start(0.025, 0.005, 0);

        int ticker = 0;
        double startAngle = rotation;
        int directionChanges = 0;
        boolean lastPositiveState = true;
        double rotationPower = 0;
        ElapsedTime dt = new ElapsedTime();

        double correctTime = 0;

        while (ticker < cycles && Op.opModeIsActive()) {
            ticker++;
            rotationPower = rotationPID_test.Loop(angle, rotation);
            rotationPower = rotationPower / (360);//rotationSpeed * Math.abs(startAngle - angle));
            rotationPower += (0.15 * (rotationPower > 0 ? 1 : -1));
            Op.telemetry.addData("Error ", rotationPID_test.error);
            Op.telemetry.addData("Last Error  ", rotationPID_test.lastError);
            Op.telemetry.addData("Derivative ", rotationPID_test.derivative);
            Op.telemetry.addData("Integral ", rotationPID_test.integral);

            Op.telemetry.addData("TD ", rotationPID_test.deltaTime.seconds());

            Op.telemetry.addData("Rotation ", rotation);
            Op.telemetry.addData("rotationPower ", rotationPower);
            Op.telemetry.addData("rotationSpeed ", rotationSpeed);
            Op.telemetry.addData("yeets", directionChanges);
            Op.telemetry.update();
            RotateSimple(rotationPower * rotationSpeed);

            if (lastPositiveState != rotationPower > 0) {
                directionChanges++;
                lastPositiveState = rotationPower > 0;
            }

//            if (rotationPID_test.error < 5) {
//                correctTime += dt.seconds();
//            }
//
//            if (correctTime > 0.25) {
//                break;
//            }

            if (directionChanges > 3) {
                ticker += cycles * 2;
                Op.telemetry.addData("Rotation ended", directionChanges);
                Op.telemetry.update();
            }
            dt.reset();
        }

        SetPowerDouble4(0, 0, 0, 0, 0);
    }

    public void RotatePID(double angle, double rotationSpeed, int cycles, double p, double i, double d) {

//        rotationPID_test.Start(3, 0.21, 0.69);
        rotationPID_test.Start(p, i, d);
//        rotationPID_test.Start(0.025, 0.005, 0);

        int ticker = 0;
        double startAngle = rotation;
        int directionChanges = 0;
        boolean lastPositiveState = true;

        while (ticker < cycles && Op.opModeIsActive()) {
            ticker++;
            double rotationPower = rotationPID_test.Loop(angle, rotation);
            rotationPower = rotationPower / (360);//rotationSpeed * Math.abs(startAngle - angle));
            rotationPower += (0.01 * (rotationPower > 0 ? 1 : -1));
            Op.telemetry.addData("Error ", rotationPID_test.error);
            Op.telemetry.addData("Last Error  ", rotationPID_test.lastError);
            Op.telemetry.addData("Derivative ", rotationPID_test.derivative);
            Op.telemetry.addData("Integral ", rotationPID_test.integral);

            Op.telemetry.addData("TD ", rotationPID_test.deltaTime.seconds());

            Op.telemetry.addData("Rotation ", rotation);
            Op.telemetry.addData("rotationPower ", rotationPower);
            Op.telemetry.addData("rotationSpeed ", rotationSpeed);
            Op.telemetry.addData("yeets", directionChanges);
            Op.telemetry.update();
            RotateSimple(rotationPower * rotationSpeed);

            if (lastPositiveState != rotationPower > 0) {
                directionChanges++;
                lastPositiveState = rotationPower > 0;
            }

            if (directionChanges > 5) {
                ticker += cycles * 2;
            }

        }

        SetPowerDouble4(0, 0, 0, 0, 0);
    }

    /**
     * @param v          this is the vector that represents our wheels power! Create a new Double4 like so:
     *                   new Double4(x,y,z,w)
     *                   <p>
     *                   See RobotConfiguration for more information
     * @param multiplier the coefficient of 'v'
     */

    public void SetPowerDouble4(Double4 v, double multiplier) {
        driveManager.frontLeft.setPower(v.x * multiplier);
        driveManager.frontRight.setPower(v.y * multiplier);
        driveManager.backLeft.setPower(v.z * multiplier);
        driveManager.backRight.setPower(v.w * multiplier);
    }

    public void SetPersistentVector(Double2 vector, double imu) {

    }

    public void SetPersistentRotation(double relativeAngle) {
    }


    public void SetPowerDouble4(double x, double y, double z, double w, double multiplier) {
        Double4 v = new Double4(x, y, z, w);

        driveManager.frontLeft.setPower(v.x * multiplier);
        driveManager.frontRight.setPower(v.y * multiplier);
        driveManager.backLeft.setPower(v.z * multiplier);
        driveManager.backRight.setPower(v.w * multiplier);

//        backRight.setPower(v.x * multiplier);
//        frontLeft.setPower(v.w * multiplier);
//        frontRight.setPower(v.z * multiplier);
//        backLeft.setPower(v.y * multiplier);
    }

    public void SetDriveMode(DcMotor.RunMode mode) {
        driveManager.frontLeft.setMode(mode);
        driveManager.backLeft.setMode(mode);
        driveManager.frontRight.setMode(mode);
        driveManager.backRight.setMode(mode);
    }

    public void SetRelitiveEncoderPosition(double delta) {

        driveManager.frontLeft.setTargetPosition(driveManager.frontLeft.getCurrentPosition() + (int) delta);
        driveManager.backLeft.setTargetPosition(driveManager.backLeft.getCurrentPosition() + (int) delta);
        driveManager.frontRight.setTargetPosition(driveManager.frontRight.getCurrentPosition() + (int) delta);
        driveManager.backRight.setTargetPosition(driveManager.backRight.getCurrentPosition() + (int) delta);
    }

    //Returns IMU rotation on the zed axies
    public double GetRotation() {
        //returns the threaded rotation values for speeeed
        return rotation;
    }

    //Returns true if any wheels are currently busy
    public boolean WheelsBusy() {
        return driveManager.frontRight.isBusy() || driveManager.frontLeft.isBusy() || driveManager.backLeft.isBusy() || driveManager.backRight.isBusy();
    }
    //</editor-fold>

    //Drive forward a set distance at a set speed, distance is measured in CM
    public void DriveByDistance(double speed, double distance) {

        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetRelitiveEncoderPosition((RobotConfiguration.wheel_ticksPerRotation / RobotConfiguration.wheel_circumference) * distance);
        SetPowerDouble4(1, 1, 1, 1, speed);
        SetDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        Op.telemetry.addData("Driving by distance ", distance * ((RobotConfiguration.wheel_circumference * RobotConfiguration.wheel_ticksPerRotation)));
        Op.telemetry.update();
        while (Op.opModeIsActive() && WheelsBusy()) {
            Op.telemetry.addData("Wheel Busy", "");
            Op.telemetry.addData("Wheel Front Right Postion", driveManager.frontRight.getCurrentPosition());
            Op.telemetry.addData("Wheel Front Right Target", driveManager.frontRight.motor.getTargetPosition());
            Op.telemetry.update();

            if (!Op.opModeIsActive()) {
                break;
            }
            //Wait until we are at our target distance
        }

        Op.telemetry.addData("Target Reached", "");
        Op.telemetry.update();

        //Stop motors
        SetPowerDouble4(0, 0, 0, 0, 0);

        //Set up for normal driving
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Returns the distance using a sensor group
    public double GetDistance(RobotWallTrack.groupID group, DistanceUnit unit) {
        return wallTrack.sensorIDGroupPairs.get(group).getDistanceAverage(unit);
    }

}
