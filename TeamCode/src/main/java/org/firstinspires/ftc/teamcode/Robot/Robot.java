package org.firstinspires.ftc.teamcode.Robot;

import android.content.Context;
import android.renderscript.Double2;
import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bDataManger;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Hardware.bIMU;
import org.firstinspires.ftc.teamcode.R;

import java.util.concurrent.atomic.AtomicBoolean;

//TODO: clean up the canmove system
public class Robot extends Thread {

    //Static instance. Only have one robot at a time and access it from here (THERE CAN BE ONLY ONE)
    public static Robot instance;

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

//    public DcMotor frontLeft;
//    public DcMotor frontRight;
//    public DcMotor backLeft;
//    public DcMotor backRight;

//    //Used for testing
//    public Servo gripServo;
//
//    //Used for testing
//    public DcMotor armWintch;

    public LinearOpMode Op;
//    public OpMode LinearOpMode;

    //If our thread is running, using atomics to avoid thread conflicts. Might not be completely necessary
    AtomicBoolean threadRunning = new AtomicBoolean();


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

            threadTimer += threadDeltaTime.seconds();
//            Op.telemetry.update();

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
     * @param movementAngle The angle (relative to the phoneside of the bot) that we want to move along, try to keep its magnitude under 180
     * @param movementSpeed How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void MoveSimple(double movementAngle, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementAngle);
        SetPowerDouble4(v, movementSpeed);
    }

    /**
     * Uses
     *
     * @param movementVector The vector (relative to the phoneside of the bot) that we want to move along
     * @param movementSpeed  How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void MoveSimple(Double2 movementVector, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementVector);
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
        rotationPID_test.Start(3, 0.40, 0.2);
//        rotationPID_test.Start(1, 0.075, 0.022);

//        rotationPID_test.Start(3, 0.21, 0.69);
//        rotationPID_test.Start(0.5, 0.075, 0.015);
//        rotationPID_test.Start(1, 0.25, 0.035);
//        rotationPID_test.Start(0.025, 0.005, 0);

        int ticker = 0;
        double startAngle = rotation;
        int directionChanges = 0;
        boolean lastPositiveState = true;

        while (ticker < cycles && Op.opModeIsActive()) {
            ticker++;
            double rotationPower = rotationPID_test.Loop(angle, rotation);
            rotationPower = rotationPower / (360);//rotationSpeed * Math.abs(startAngle - angle));
            rotationPower += (0.5 * (rotationPower > 0 ? 1 : -1));
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
            rotationPower += 0.25 * (rotationPower > 0 ? 1 : -1);
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

        driveManager.frontLeft.setTargetPosition(driveManager.frontLeft.getCurrentPosition());
        driveManager.backLeft.setTargetPosition(driveManager.backLeft.getCurrentPosition());
        driveManager.frontRight.setTargetPosition(driveManager.frontRight.getCurrentPosition());
        driveManager.backRight.setTargetPosition(driveManager.backRight.getCurrentPosition());
    }

    //Returns IMU rotation on the zed axies
    public double GetRotation() {
        //returns the threaded rotation values for speeeed
        return rotation;
    }

    //Returns true if any wheels are currently busy
    public boolean WheelsBusy() {
        return driveManager.frontRight.isBusy() && driveManager.frontLeft.isBusy() && driveManager.backLeft.isBusy() && driveManager.backRight.isBusy();
    }
    //</editor-fold>

    //Drive forward a set distance at a set speed, distance is measured in CM
    public void DriveByDistance(double speed, double distance) {
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetPowerDouble4(1, 1, 1, 1, speed);
        SetRelitiveEncoderPosition(distance * ((RobotConfiguration.wheel_circumference * RobotConfiguration.wheel_ticksPerRotation) * RobotConfiguration.wheel_GearCoefficient));

        while (WheelsBusy()) {
            //Wait until we are at our target distance
        }

        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //wip
//    public void SetRotation(double rotation, double threshold, double speed) {
//        double difference = GetRotation() - rotation;
//
//        //Rotate to 'rotation'
//        while (Math.abs(difference) > threshold) {
//            Rotate(GetRotation() + difference, speed);
//        }
//
//        //Let the bot settle for 200ms
//        try {
//            Thread.sleep(200);
//        } catch (InterruptedException e) {
//
//        }
//
//        //Check the rotation again
//        difference = rotation - GetRotation();
//
//        //Settle again
//        while (Math.abs(difference) > threshold) {
//            difference = GetRotation() - rotation;
//
//            Rotate(GetRotation() + difference, speed / 3);
//        }
//    }

    //Experimental version of set rotation without sleep (PID?)
//    public void SetRotationExperimental(double rotation, double threshold, double speed) {
//        double difference = GetRotation() - rotation;
//        double initialDifference = difference;
//        double rotationSpeed;
//
//
//        //Rotate to 'rotation'
//        while (Math.abs(difference) > threshold) {
//            //Check the rotation again
//            difference = GetRotation() - rotation;
//            rotationSpeed = bMath.Lerp(-speed, speed, (difference) / initialDifference);
//
//            Op.telemetry.addData("diff ", difference);
//            Op.telemetry.addData("rotationSpeed ", rotationSpeed * 2);
//
//            Op.telemetry.update();
//
//            Rotate(-1, rotationSpeed);
//        }
//
//    }

//    //Experimental version of set rotation without sleep (PID?)
//    public void SetRotationPID(double rotation, double threshold, double P, double I, double D) {
//
//        double difference = rotation - GetRotation();
//        double lastDifference = rotation - GetRotation();
//        double initialDifference = difference;
//        double rotationSpeed;
//
//        double integral = 0;
//        double derivative = 0;
//
//        DeltaTime dt = new DeltaTime();
//
//        //Rotate to 'rotation'
//        while (Math.abs(difference) > threshold) {
//            dt.Start();
//
//            //Check the rotation again
//            difference = rotation - GetRotation();
//
//            integral += difference * dt.deltaTime();
//            derivative = (difference - lastDifference) / dt.deltaTime();
//
//            rotationSpeed = difference;
//
//            Op.telemetry.addData("diff ", difference);
//            Op.telemetry.addData("rotationSpeed ", rotationSpeed * 2);
//
//            Op.telemetry.update();
//
//            Rotate(1, rotationSpeed);
//
//            lastDifference = (P * difference) + (I * integral) + (D * derivative);
//
//            dt.Stop();
//
//        }
//
//
//    }

//    public void CalibrateWheels() {
//        driveManager.calibrating = true;
//
//        SetPowerDouble4(1, 1, 1, 1, 1);
//
//
//        driveManager.calibrating = false;
//
//    }


}
