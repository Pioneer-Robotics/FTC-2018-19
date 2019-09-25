package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.locks.Condition;

class HardwareInfinity1 extends Thread {
    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();
    private double sq2 = Math.sqrt(2);

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    BNO055IMU imu;
    LinearOpMode Op;

    //Toggles Op.telemetry output
    boolean enableTelemetry = false;

    void init(HardwareMap ahwMap, LinearOpMode O) {
        Op = O;
        frontLeft = ahwMap.get(DcMotor.class, "Front Left");
        frontRight = ahwMap.get(DcMotor.class, "Front Right");
        backLeft = ahwMap.get(DcMotor.class, "Back Left");
        backRight = ahwMap.get(DcMotor.class, "Back Right");
        imu = ahwMap.get(BNO055IMU.class, "imu");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        imu.initialize(IParameters);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

    }

    void MecDrive(double centimeters, double angle1, double speed) {

        //initialize target variables for encoderDrive
        double distance = centimeters * 1000 / 22.55;
        double leftDiagTarget;
        double rightDiagTarget;
        double dynamicLeftDiagTarget;
        double dynamicRightDiagTarget;
        double angle = Math.toRadians(-angle1);
        boolean leftDisable = false;
        boolean rightDisable = false;

        //reset motors, ensuring they are completely stopped while doing so.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle));

        rightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle));

        dynamicLeftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle));

        dynamicRightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle));

        if (Math.abs(leftDiagTarget) < 0.01) leftDisable = true;
        if (Math.abs(rightDiagTarget) < 0.01) rightDisable = true;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftDiagPower = 0;
        double rightDiagPower = 0;

        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {
        }

        leftDiagPower = ((-Math.sin(angle) * speed + Math.cos(angle) * speed) / sq2);
        rightDiagPower = ((-Math.sin(angle) * speed - Math.cos(angle) * speed) / sq2);

        frontLeft.setPower(leftDiagPower);
        frontRight.setPower(rightDiagPower);
        backLeft.setPower(rightDiagPower);
        backRight.setPower(leftDiagPower);
        if (enableTelemetry) {
            Op.telemetry.addData("LFE:", frontLeft.getCurrentPosition());
            Op.telemetry.addData("RFE:", frontRight.getCurrentPosition());
            Op.telemetry.addData("LDT:", leftDiagTarget);
            Op.telemetry.addData("RDT:", rightDiagTarget);
            Op.telemetry.addData("DLDT:", dynamicLeftDiagTarget);
            Op.telemetry.addData("DRDT:", dynamicRightDiagTarget);
            Op.telemetry.addData("C1:", Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget));
            Op.telemetry.addData("C2:", Math.abs(frontRight.getCurrentPosition() - rightDiagTarget));
            Op.telemetry.update();
        }
        while ((leftDisable || (10 < Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget))) &&
                (rightDisable || (10 < Math.abs(frontRight.getCurrentPosition() - rightDiagTarget))) &&
                (leftDisable || ((Math.abs(dynamicLeftDiagTarget) + 10) >= Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget))) &&
                (rightDisable || ((Math.abs(dynamicRightDiagTarget) + 10) >= Math.abs(frontRight.getCurrentPosition() - rightDiagTarget)))) {


            dynamicLeftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle)) - frontLeft.getCurrentPosition();
            dynamicRightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle)) - frontRight.getCurrentPosition();
            if (Op.isStopRequested()) { //prevents crashes when emergency stop is activated
                frontLeft.setPower(0);
                frontRight.setPower(0);
                return;
            }
            if (enableTelemetry) {
                Op.telemetry.addData("LFE:", frontLeft.getCurrentPosition());
                Op.telemetry.addData("RFE:", frontRight.getCurrentPosition());
                Op.telemetry.addData("LDT:", leftDiagTarget);
                Op.telemetry.addData("RDT:", rightDiagTarget);
                Op.telemetry.addData("DLDT:", dynamicLeftDiagTarget);
                Op.telemetry.addData("DRDT:", dynamicRightDiagTarget);
                Op.telemetry.addData("C1:", Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget));
                Op.telemetry.addData("C2:", Math.abs(frontRight.getCurrentPosition() - rightDiagTarget));
                Op.telemetry.update();
            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException ignored) {
        }


    }

    //Testing version of mec drive that does not use thread sleepingingningingnggn
    //What could go wrong!
    void MecDriveWoke(double centimeters, double angle1, double speed) {

        //initialize target variables for encoderDrive
        double distance = centimeters * 1000 / 22.55;
        double leftDiagTarget;
        double rightDiagTarget;
        double dynamicLeftDiagTarget;
        double dynamicRightDiagTarget;
        double angle = Math.toRadians(-angle1);
        boolean leftDisable = false;
        boolean rightDisable = false;

        //reset motors, ensuring they are completely stopped while doing so.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle));

        rightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle));

        dynamicLeftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle));

        dynamicRightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle));

        if (Math.abs(leftDiagTarget) < 0.01) leftDisable = true;
        if (Math.abs(rightDiagTarget) < 0.01) rightDisable = true;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftDiagPower = 0;
        double rightDiagPower = 0;

//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException ignored) {
//        }

        leftDiagPower = ((-Math.sin(angle) * speed + Math.cos(angle) * speed) / sq2);
        rightDiagPower = ((-Math.sin(angle) * speed - Math.cos(angle) * speed) / sq2);

        frontLeft.setPower(leftDiagPower);
        frontRight.setPower(rightDiagPower);
        backLeft.setPower(rightDiagPower);
        backRight.setPower(leftDiagPower);

        while ((leftDisable || (10 < Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget))) &&
                (rightDisable || (10 < Math.abs(frontRight.getCurrentPosition() - rightDiagTarget))) &&
                (leftDisable || ((Math.abs(dynamicLeftDiagTarget) + 10) >= Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget))) &&
                (rightDisable || ((Math.abs(dynamicRightDiagTarget) + 10) >= Math.abs(frontRight.getCurrentPosition() - rightDiagTarget)))) {


            dynamicLeftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle)) - frontLeft.getCurrentPosition();
            dynamicRightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle)) - frontRight.getCurrentPosition();
            if (Op.isStopRequested()) { //prevents crashes when emergency stop is activated
                frontLeft.setPower(0);
                frontRight.setPower(0);
                return;
            }

        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException ignored) {
//        }


    }

    void MecDriveStart(double centimeters, double angle1, double speed, boolean resetEncoders) {

        //initialize target variables for encoderDrive
        double distance = centimeters * 1000 / 22.55;
        double leftDiagTarget;
        double rightDiagTarget;
        double dynamicLeftDiagTarget;
        double dynamicRightDiagTarget;
        double angle = Math.toRadians(-angle1);
        boolean leftDisable = false;
        boolean rightDisable = false;


        if (resetEncoders) {
            //reset motors, ensuring they are completely stopped while doing so.
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle));

        rightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle));

        dynamicLeftDiagTarget = distance * (Math.sin(-angle) + Math.cos(-angle));

        dynamicRightDiagTarget = distance * (Math.sin(-angle) - Math.cos(-angle));

        if (Math.abs(leftDiagTarget) < 0.01) leftDisable = true;
        if (Math.abs(rightDiagTarget) < 0.01) rightDisable = true;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftDiagPower = 0;
        double rightDiagPower = 0;


        leftDiagPower = ((-Math.sin(angle) * speed + Math.cos(angle) * speed) / sq2);
        rightDiagPower = ((-Math.sin(angle) * speed - Math.cos(angle) * speed) / sq2);

        frontLeft.setPower(leftDiagPower);
        frontRight.setPower(rightDiagPower);
        backLeft.setPower(rightDiagPower);
        backRight.setPower(leftDiagPower);
        if (enableTelemetry) {
            Op.telemetry.addData("LFE:", frontLeft.getCurrentPosition());
            Op.telemetry.addData("RFE:", frontRight.getCurrentPosition());
            Op.telemetry.addData("LDT:", leftDiagTarget);
            Op.telemetry.addData("RDT:", rightDiagTarget);
            Op.telemetry.addData("DLDT:", dynamicLeftDiagTarget);
            Op.telemetry.addData("DRDT:", dynamicRightDiagTarget);
            Op.telemetry.addData("C1:", Math.abs(frontLeft.getCurrentPosition() - leftDiagTarget));
            Op.telemetry.addData("C2:", Math.abs(frontRight.getCurrentPosition() - rightDiagTarget));
            Op.telemetry.update();
        }

    }

    public void TestNewMovement(double movementAngle, double rotationAngleDelta, double speed) {
        Double4 v = bMath.getMecMovement(movementAngle, rotationAngleDelta);
        SetPowerDouble4(v, speed);
    }

    public void SetPowerDouble4(Double4 v, double multiplier) {
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
