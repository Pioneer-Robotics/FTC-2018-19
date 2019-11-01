package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;
import android.renderscript.Double4;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

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

    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();
    public BNO055IMU imu;

    //Hardware!
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    //Used for testing
    public Servo armServo;

    OpMode Op;

    Boolean running;


    public void init(HardwareMap hardwareMap, OpMode opmode) {
        //Set up the instance (safety checks might be a good idea at some point)
        instance = this;

        //Set the opmode
        Op = opmode;


        //Find the motors
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");

        //Set the left wheels to run backwards because of math?

        //Those wheels are reversed so power 1,1,1,1 moves us forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //Init the motors for use. NTS: If you don't do this the robot does not like to move with math
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Set up the IMU(s)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        imu.initialize(IParameters);


        opmode.telemetry.addData("Setting up wall track", "");
        opmode.telemetry.update();
        //Set up the wall tracker, this uses ALL the lasers so make sure they all work before running this
        wallTrack.Start(opmode);

        opmode.telemetry.addData("track", "");
        opmode.telemetry.update();


        //Starts the 'run' thread
        start();

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
        }
    }


    public void BackgroundRotation() {

        //Updates the current rotation
        rotation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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

    public void Rotate(double angle, double rotationSpeed) {
        Double4 v = bMath.getMecRotation(angle, rotationSpeed);
        SetPowerDouble4(v, rotationSpeed);
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
        frontLeft.setPower(v.x * multiplier);
        frontRight.setPower(v.y * multiplier);
        backLeft.setPower(v.z * multiplier);
        backRight.setPower(v.w * multiplier);
    }

    public void SetPersistentVector(Double2 vector, double imu) {

    }

    public void SetPersistentRotation(double relativeAngle) {
    }


    public void SetPowerDouble4(double x, double y, double z, double w, double multiplier) {
        Double4 v = new Double4(x, y, z, w);

        frontLeft.setPower(v.x * multiplier);
        frontRight.setPower(v.y * multiplier);
        backLeft.setPower(v.z * multiplier);
        backRight.setPower(v.w * multiplier);
    }

    public void SetDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }

    //Returns IMU rotation on the zed axies
    public double GetRotation() {
        //returns the threaded rotation values for speeeed
        return rotation;
    }
    //</editor-fold>


    //wip
    public void SetRotation(double rotation, double threshold, double speed) {
        double difference = GetRotation() - rotation;

        //Rotate to 'rotation'
        while (Math.abs(difference) > threshold) {
            Rotate(GetRotation() + difference, speed);
        }

        //Let the bot settle for 200ms
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {

        }

        //Check the rotation again
        difference = rotation - GetRotation();

        //Settle again
        while (Math.abs(difference) > threshold) {
            difference = GetRotation() - rotation;

            Rotate(GetRotation() + difference, speed / 3);
        }
    }

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

    //Experimental version of set rotation without sleep (PID?)
    public void SetRotationPID(double rotation, double threshold, double P, double I, double D) {

        double difference = rotation - GetRotation();
        double lastDifference = rotation - GetRotation();
        double initialDifference = difference;
        double rotationSpeed;

        double integral = 0;
        double derivative = 0;

        DeltaTime dt = new DeltaTime();

        //Rotate to 'rotation'
        while (Math.abs(difference) > threshold) {
            dt.Start();

            //Check the rotation again
            difference = rotation - GetRotation();

            integral += difference * dt.deltaTime();
            derivative = (difference - lastDifference) / dt.deltaTime();

            rotationSpeed = difference;

            Op.telemetry.addData("diff ", difference);
            Op.telemetry.addData("rotationSpeed ", rotationSpeed * 2);

            Op.telemetry.update();

            Rotate(1, rotationSpeed);

            lastDifference = (P * difference) + (I * integral) + (D * derivative);

            dt.Stop();

        }


    }
}
