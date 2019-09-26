package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

class HardwareInfinityMec extends Thread {
    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    BNO055IMU imu;

    LinearOpMode Op;


    void init(HardwareMap ahwMap, LinearOpMode O) {
        Op = O;

        frontLeft = ahwMap.get(DcMotor.class, "Front Left");
        frontRight = ahwMap.get(DcMotor.class, "Front Right");
        backLeft = ahwMap.get(DcMotor.class, "Back Left");
        backRight = ahwMap.get(DcMotor.class, "Back Right");


        imu = ahwMap.get(BNO055IMU.class, "imu");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";

        imu.initialize(IParameters);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

    }


    public void MoveSimple(double movementAngle, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(movementAngle);
        SetPowerDouble4(v, movementSpeed);
    }

    private void SetPowerDouble4(Double4 v, double multiplier) {
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
}
