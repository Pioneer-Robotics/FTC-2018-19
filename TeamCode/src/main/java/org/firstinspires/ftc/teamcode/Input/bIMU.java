package org.firstinspires.ftc.teamcode.Input;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


//Used in place of BNO055IMU, it takes the average of both IMU's for readings
public class bIMU {

    private BNO055IMU imu_0;
    private BNO055IMU imu_1;

    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();

    OpMode op;

    public void Start(OpMode opMode, String imu_0_name, String imu_1_name) {
        op = opMode;

        //Start up the first IMU
        imu_0 = opMode.hardwareMap.get(BNO055IMU.class, imu_0_name);
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU 0";

        imu_0.initialize(IParameters);


        //Start up the first IMU
        imu_1 = opMode.hardwareMap.get(BNO055IMU.class, imu_1_name);
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU 1";

        imu_1.initialize(IParameters);
    }


    //Returns the average of both IMU rotations
    public double getRotation(AngleUnit angleUnit) {
        op.telemetry.addData("IMU 0: ", imu_0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle);
        op.telemetry.addData("IMU 1: ", imu_1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle);

        return (imu_0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle + imu_1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle) / 2;
    }

}
