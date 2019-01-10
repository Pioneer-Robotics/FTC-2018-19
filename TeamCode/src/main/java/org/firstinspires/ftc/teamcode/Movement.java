package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class Movement extends Thread {
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private BNO055IMU imu;
    private LinearOpMode Op;
    private ElapsedTime runtime;
    private double COUNTS_PER_INCH;
    private double speedG;
    private double angleG;
    private double leftCMG;
    private double rightCMG;
    private double timeoutSG;
    private int mode;

    int margin = 7;

    void init(DcMotor motL, DcMotor motR, BNO055IMU im, LinearOpMode O, ElapsedTime run, double CPI) {
        motorLeft = motL;
        motorRight = motR;
        imu = im;
        Op = O;
        runtime = run;
        COUNTS_PER_INCH = CPI;
    }

    void angleTurn(double speed, double angle, boolean backgrnd) {
        double targetAngle;
        if (Op.opModeIsActive()) {
            if (backgrnd) {
                angleG = angle;
                speedG = speed;
                mode = 1;
                start();

            }
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetAngle = angle + angles.firstAngle + 180;
            Op.telemetry.clearAll();
            while (Math.abs(angles.firstAngle + 180 - targetAngle) > margin * speed && (360 - Math.abs(angles.firstAngle + 180 - targetAngle)) > margin * speed) {
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                if (angle > 0) {
                    motorLeft.setPower(Math.abs(speed));
                    motorRight.setPower(-Math.abs(speed));
                } else {
                    motorLeft.setPower(-Math.abs(speed));
                    motorRight.setPower(Math.abs(speed));
                }
                Op.telemetry.addData("Error:", "%.5f", Math.abs(angles.firstAngle + 180 - targetAngle));
                Op.telemetry.addData("Margin:", "%.5f", margin * speed);
                Op.telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
                Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
                Op.telemetry.addData("target:", "%.5f", targetAngle);
                Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
                Op.telemetry.update();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                if (Math.abs(angles.firstAngle + 180 - targetAngle) < margin * speed || (360 - Math.abs(angles.firstAngle + 180 - targetAngle)) < margin * speed) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
            }
            Op.telemetry.addData("Finished", "!");
            Op.telemetry.update();
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }
    void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS, boolean backgrnd) {
        int newLeftTarget;
        int newRightTarget;
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double initAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (Op.opModeIsActive()) {
            if (backgrnd) {
                speedG = speed;
                leftCMG = leftCM;
                rightCMG = rightCM;
                timeoutSG = timeoutS;
                mode = 2;
                start();

            }
            newLeftTarget = motorLeft.getCurrentPosition() - (int) (leftCM * COUNTS_PER_INCH);
            newRightTarget = motorLeft.getCurrentPosition() - (int) (rightCM * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            while (Op.opModeIsActive() && (runtime.seconds() < timeoutS) && motorLeft.isBusy() && motorRight.isBusy())
            {
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                Op.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                Op.telemetry.addData("Path2", "Running at %7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                /*if (robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > initAng+10
                        || robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle < initAng-10) {

                    int mLt = robot.motorLeft.getCurrentPosition();
                    int mRt = robot.motorRight.getCurrentPosition();
                    angleTurn(0.5, initAng);
                    robot.motorLeft.setTargetPosition(newLeftTarget+(robot.motorLeft.getCurrentPosition()-mLt));
                    robot.motorRight.setTargetPosition(newRightTarget+(robot.motorRight.getCurrentPosition()-mRt));
                }*/
                Op.telemetry.update();
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
            }

            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Op.sleep(250);
        }
    }
    public void run() {
        if (mode == 1) {
            angleTurn(speedG,angleG,false);
        } else if (mode == 2) {
            encoderDrive(speedG, leftCMG, rightCMG, timeoutSG,false);
        } else {
            return;
        }
    }
}
