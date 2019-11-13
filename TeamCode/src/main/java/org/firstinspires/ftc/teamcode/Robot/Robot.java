package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;
import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Hardware.bIMU;

//TODO: clean up the canmove system
public class Robot extends Thread {

    //Static instance. Only have one robot at a time and access it from here (THERE CAN BE ONLY ONE)
    public static Robot instance;

    //Our wee little wall tracker
    public RobotWallTrack wallTrack = new RobotWallTrack();

    //This delta time is only for the navigation helper thread
    public DeltaTime deltaTime = new DeltaTime();

    //Timer for canmove
    public double canMoveTimer;

    //If we are allowed to move
    public Boolean canMove() {
        return canMoveTimer < 0;
    }

    //The current IMU rotation, threaded
    double rotation;

    public bIMU imu;

    //Hardware!
    public RobotDriveManager driveManager;

//    public DcMotor frontLeft;
//    public DcMotor frontRight;
//    public DcMotor backLeft;
//    public DcMotor backRight;

//    //Used for testing
//    public Servo gripServo;
//
//    //Used for testing
//    public DcMotor armWintch;

    public OpMode Op;

    Boolean running;


    public void init(HardwareMap hardwareMap, OpMode opmode) {

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
        SetDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        bTelemetry.Print("Robot start up successful. Ready to operate!");

    }


    //Threaded run method, right now this is just for IMU stuff, at some point we might put some avoidance stuff in here (background wall tracking?) (average out 2IMU's for extra strain of the thread?)
    public void run() {
        running = true;

        double closestLaserDistance = 0;
        double laserAngle = 0;

        while (running) {
//            deltaTime.Start();
//
//            //region Avoidance
//            closestLaserDistance = 50000;
//
//            //Finds the laser nearest to an object and which way its facing
//            for (bDistanceSensor laser : lasers) {
//                if (laser.distance < closestLaserDistance) {
//                    closestLaserDistance = laser.distance;
//                    laserAngle = laser.angle;
//                }
//            }
//
//            //GOAL: If the nearest laser is within 100 of hitting something then suspend movement in that direction briefly
//            //Right now just turn off the motors to avoid collision and wait for the object to move
//            if (closestLaserDistance < 100) {
//                canMoveTimer = 2.5;
//            }
//
//            //Count down the can move timer
//            canMoveTimer -= deltaTime.deltaTime();
//
//            if (!canMove()) {
//                SetPowerDouble4(0, 0, 0, 0, 0);
//            }
//            //endregion
//
//
//            deltaTime.Stop();

            //Update our 'rotation' value
            BackgroundRotation();

            //Update our driver
            driveManager.Update();
        }
    }


    public void BackgroundRotation() {
        //Updates the current rotation
        rotation = imu.getRotation(AngleUnit.DEGREES);
    }


    public void Stop() {
        running = false;
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
        SetPowerDouble4(v, rotationSpeed);
    }

    PID rotationPID_test = new PID();

    public void RotatePID(double angle, double rotationSpeed, int cycles) {

        rotationPID_test.Start(0.1f, 0, 0);

        int ticker = 0;

        while (ticker < cycles) {
            ticker++;
            double rotationPower = rotationPID_test.Loop(angle, rotation);
            RotateSimple(rotationPower * rotationSpeed);
        }

    }

    /**
     * @param v          this is the vector that represents our wheels power! Create a new Double4 like so:
     *                   new Double4(x,y,z,w)
     *                   <p>
     *                   X____Y
     *                   | ^^ |
     *                   |    ||
     *                   |____|
     *                   Z    W
     *                   robot (not up to date with newest drive train)
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

    //Returns IMU rotation on the zed axies
    public double GetRotation() {
        //returns the threaded rotation values for speeeed
        return rotation;
    }
    //</editor-fold>


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
}
