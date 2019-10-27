package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

//TODO: clean up the canmove system
public class Robot extends Thread {

    //The eight lasers of navigationness! Right now we only have 6 so its slightly less impressive I suppose
    public bDistanceSensor[] lasers = new bDistanceSensor[6];

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

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    public BNO055IMU imu;

    OpMode Op;

    Boolean running;

    public void init(HardwareMap hardwareMap, OpMode opmode) {
        Op = opmode;

        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");


        //IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";

        imu.initialize(IParameters);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //Current lasers on bot, all facing away from the phone
//        lasers[0] = new bDistanceSensor(opmode, "sensor 315", -45);
//        lasers[1] = new bDistanceSensor(opmode, "sensor0", 0);
//        lasers[2] = new bDistanceSensor(opmode, "sensor45", 45);
//
//
//        lasers[3] = new bDistanceSensor(opmode, "sensor45", 45);
//        lasers[4] = new bDistanceSensor(opmode, "sensor90", 90);
//        lasers[5] = new bDistanceSensor(opmode, "sensor135", 135);

        //Start 'run'
        start();

    }


    //Threaded run method
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

            BackgroundRotation();
        }
    }


    public void BackgroundRotation() {
        //Updates the current rotation
        rotation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Display debug stuffs

//        Op.telemetry.addData("IMU rotation ", rotation);
//        Op.telemetry.update();
    }


    public void Stop() {
        running = false;
    }


    //<editor-fold desc="Movement">
    public void MoveSimple(double movementAngle, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementAngle - 90);
        SetPowerDouble4(v, movementSpeed);
    }

    public void MoveComplex(double movementAngle, double movementSpeed, double angle) {
        Double4 v = bMath.getMecMovement(movementAngle - 90, angle - 90);
        SetPowerDouble4(v, movementSpeed);
    }

    public void Rotate(double angle, double rotationSpeed) {
        Double4 v = bMath.getMecRotation(angle - 90, rotationSpeed - 90);
        SetPowerDouble4(v, rotationSpeed);
    }

    public void SetPowerDouble4(Double4 v, double multiplier) {
        frontLeft.setPower(v.x * multiplier);
        frontRight.setPower(v.y * multiplier);
        backLeft.setPower(v.z * multiplier);
        backRight.setPower(v.w * multiplier);
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


    public class bDistanceSensor {

        //The actual sensor
        DistanceSensor distanceSensor;

        //The angle that this sensors laser goes relative to the phone facing side  (-180 <=> 180)
        double angle = 0;

        public double distance;

        public double distanceSmoothed;

        private final double smoothingStep = 1;

        public bDistanceSensor(OpMode op, String _sensor, double _angle) {
            distanceSensor = op.hardwareMap.get(DistanceSensor.class, _sensor);
            angle = _angle;
        }

        //Called externally to tick the senor smoothing and update distance values
        public void Update() {
            distance = distanceSensor.getDistance(DistanceUnit.CM);
            distanceSmoothed = bMath.MoveTowards(distanceSmoothed, distance, smoothingStep);
        }

    }

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
    public void SetRotationPID(double rotation, double threshold) {

        //Tuning values
        double P = 1, I = 1, D = 1;

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
