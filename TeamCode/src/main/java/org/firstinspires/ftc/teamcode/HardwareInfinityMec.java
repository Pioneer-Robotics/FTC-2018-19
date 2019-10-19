package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//TODO: clean up the canmove system
class HardwareInfinityMec extends Thread {

    //The eight lasers of navigationness! Right now we only have 3 so its slightly less impressive I suppose
    public bDistanceSensor[] lasers = new bDistanceSensor[3];

    //This delta time is only for the navigation helper thread
    public DeltaTime deltaTime = new DeltaTime();

    //Timer for canmove
    public double canMoveTimer;

    //If we are allowed to move
    public Boolean canMove() {
        return canMoveTimer < 0;
    }

    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    BNO055IMU imu;

    OpMode Op;

    Boolean running;

    void init(HardwareMap hardwareMap, OpMode opmode) {
        Op = opmode;

        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");


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
        lasers[0] = new bDistanceSensor(opmode, "sensor180", 180);
        lasers[1] = new bDistanceSensor(opmode, "sensor225", 225);
        lasers[2] = new bDistanceSensor(opmode, "sensor135", 135);

        //Start 'run'
        start();

    }


    //Threaded run method
    public void run() {
        running = true;

        double closestLaserDistance = 0;
        double laserAngle = 0;

        while (running) {
            deltaTime.Start();

            closestLaserDistance = 50000;

            //Finds the laser nearest to an object and which way its facing
            for (bDistanceSensor laser : lasers) {
                if (laser.distance < closestLaserDistance) {
                    closestLaserDistance = laser.distance;
                    laserAngle = laser.angle;
                }
            }

            //GOAL: If the nearest laser is within 100 of hitting somthing then move away from that somthing
            //Right now just turn off the motors to avoid collision and wait for the object to move
            if (closestLaserDistance < 100) {
                canMoveTimer = 2.5;
            }

            //Count down the can move timer
            canMoveTimer -= deltaTime.deltaTime();

            if (!canMove()) {
                SetPowerDouble4(0, 0, 0, 0, 0);
            }
            deltaTime.Stop();
        }
    }


    public void MoveSimple(double movementAngle, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementAngle);
        SetPowerDouble4(v, movementSpeed);
    }

    public void MoveComplex(double movementAngle, double movementSpeed, double angle) {
        Double4 v = bMath.getMecMovement(movementAngle, angle);
        SetPowerDouble4(v, movementSpeed);
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
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void SetRotation(double rotation, double threshold) {
        while (Math.abs(GetRotation() - rotation) > threshold) {
            MoveComplex(0, 0, rotation);

        }

    }


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
}
