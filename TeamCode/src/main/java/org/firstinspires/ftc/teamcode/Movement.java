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
    void experimentalTurn(double speed, double angle, boolean backgrnd) {
        double targetAngle;
        double time;
        double maxtime = 0;
        double maxdel = 0;
        //double start;
        double spd = 0;
        int direction = 0; // -1 = cw, 1 = ccw
        double dis = 0;
        if (Op.opModeIsActive()) {
            if (backgrnd) {
                angleG = angle;
                speedG = speed;
                mode = 1;
                start();

            }
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Orientation delta = angles;
            //start = angles.firstAngle;
            targetAngle = angle + angles.firstAngle + 180;
            Op.telemetry.clearAll();
            runtime.reset();
            while (true) {
                time = runtime.milliseconds();
                if (time>maxtime) maxtime = time;
                if (Math.abs(angles.firstAngle-delta.firstAngle)> maxdel) maxdel = Math.abs(angles.firstAngle-delta.firstAngle);
                runtime.reset();
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                if ((angles.firstAngle-targetAngle)%360<(targetAngle-angles.firstAngle)%360) {
                    direction = -1;
                    dis = (angles.firstAngle-targetAngle)%360;
                } else {
                    direction = 1;
                    dis = (targetAngle-angles.firstAngle)%360;
                }
                /*if (angle > 0 /*&& (Math.abs(((angles.firstAngle-start)%360 + (targetAngle-angles.firstAngle)%360)-angle)<0.5 || Math.abs(((start-angles.firstAngle)%360 + (angles.firstAngle-targetAngle)%360)-angle)<0.5)) {
                    spd = (targetAngle-angles.firstAngle)%360/
                }*/
                spd=dis/angles.firstAngle;
                motorLeft.setPower(direction*Math.abs(speed*spd));
                motorRight.setPower(-direction*Math.abs(speed*spd));
                /*if (angle > 0) {
                    motorLeft.setPower(Math.abs(speed));
                    motorRight.setPower(-Math.abs(speed));
                } else {
                    motorLeft.setPower(-Math.abs(speed));
                    motorRight.setPower(Math.abs(speed));
                }*/
                /*Op.telemetry.addData("Error:", "%.5f", Math.abs(angles.firstAngle + 180 - targetAngle));
                Op.telemetry.addData("Margin:", "%.5f", margin * speed);
                Op.telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
                Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
                Op.telemetry.addData("target:", "%.5f", targetAngle);
                Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);*/
                Op.telemetry.addData("time: ", "%.2f",time);
                Op.telemetry.addData("delta:", "%.2f", angles.firstAngle-delta.firstAngle);
                Op.telemetry.addData("maxtime:", "%.2f", maxtime);
                Op.telemetry.addData("maxdel:", "%.2f",maxdel);
                Op.telemetry.update();
                delta = angles;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                /*if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }*/
                if (Math.abs(angles.firstAngle + 180 - targetAngle) < margin * speed || (360 - Math.abs(angles.firstAngle + 180 - targetAngle)) < margin * speed) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    break;
                }
            }
            motorLeft.setPower(0);
            motorRight.setPower(0);
            Op.telemetry.addData("Finished", "!");
            Op.telemetry.addData("Error:", "%.5f", Math.abs(angles.firstAngle + 180 - targetAngle));
            Op.telemetry.addData("Margin:", "%.5f", margin * speed);
            Op.telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
            Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
            Op.telemetry.addData("target:", "%.5f", targetAngle);
            Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
            Op.telemetry.addData("time:", "%.2f", time);
            Op.telemetry.addData("delta:", "%.2f", angles.firstAngle-delta.firstAngle);
            Op.telemetry.addData("maxtime:", "%.2f", maxtime);
            Op.telemetry.addData("maxdel:", "%.2f",maxdel);
            Op.telemetry.update();
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try {
                sleep(5000);
            } catch (InterruptedException e) {

            }
            Op.sleep(5000);
        }
    }
    void angleTurn(double speed, double angle, boolean backgrnd) {
        double targetAngle;
        double time = 0;
        double maxtime = 0;
        double maxdel = 0;
        if (Op.opModeIsActive()) {
            if (backgrnd) {
                angleG = angle;
                speedG = speed;
                mode = 1;
                start();

            }
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Orientation delta = angles;
            targetAngle = angle + angles.firstAngle + 180;
            Op.telemetry.clearAll();
            runtime.reset();
            while (true) {
                time = runtime.milliseconds();
                if (time>maxtime) maxtime = time;
                if (Math.abs(angles.firstAngle-delta.firstAngle)> maxdel) maxdel = Math.abs(angles.firstAngle-delta.firstAngle);
                runtime.reset();
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
                /*Op.telemetry.addData("Error:", "%.5f", Math.abs(angles.firstAngle + 180 - targetAngle));
                Op.telemetry.addData("Margin:", "%.5f", margin * speed);
                Op.telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
                Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
                Op.telemetry.addData("target:", "%.5f", targetAngle);
                Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);*/
                Op.telemetry.addData("time: ", "%.2f",time);
                Op.telemetry.addData("delta:", "%.2f", angles.firstAngle-delta.firstAngle);
                Op.telemetry.addData("maxtime:", "%.2f", maxtime);
                Op.telemetry.addData("maxdel:", "%.2f",maxdel);
                Op.telemetry.update();
                delta = angles;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                /*if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }*/
                if (Math.abs(angles.firstAngle + 180 - targetAngle) < margin * speed || (360 - Math.abs(angles.firstAngle + 180 - targetAngle)) < margin * speed) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    break;
                }
            }
            motorLeft.setPower(0);
            motorRight.setPower(0);
            Op.telemetry.addData("Finished", "!");
            Op.telemetry.addData("Error:", "%.5f", Math.abs(angles.firstAngle + 180 - targetAngle));
            Op.telemetry.addData("Margin:", "%.5f", margin * speed);
            Op.telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
            Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
            Op.telemetry.addData("target:", "%.5f", targetAngle);
            Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
            Op.telemetry.addData("time:", "%.2f", time);
            Op.telemetry.addData("delta:", "%.2f", angles.firstAngle-delta.firstAngle);
            Op.telemetry.addData("maxtime:", "%.2f", maxtime);
            Op.telemetry.addData("maxdel:", "%.2f",maxdel);
            Op.telemetry.update();
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try {
                sleep(5000);
            } catch (InterruptedException e) {

            }
            Op.sleep(5000);
        }
    }
    void encoderDriveOld(double speed, double leftCM, double rightCM, double timeoutS, boolean backgrnd) {
        int newLeftTarget;
        int newRightTarget;

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
            Op.telemetry.addData("Encoder Target: ", "%7d :%7d", newLeftTarget, newRightTarget);
            Op.telemetry.addData("Current Position: ", "%7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            Op.telemetry.update();
            try {
                sleep(2000);
            } catch (InterruptedException e) {

            }
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            while (Op.opModeIsActive() && (runtime.seconds() < timeoutS) && (motorLeft.isBusy() && motorRight.isBusy()))
            {
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                Op.telemetry.addData("Encoder Target: ", "%7d :%7d", newLeftTarget, newRightTarget);
                Op.telemetry.addData("Current Position: ", "%7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
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
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setPower(0);
            motorRight.setPower(0);
            try {
                sleep(2000);
            } catch (InterruptedException e) {

            }

            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Op.sleep(250);
        }
    }
    void experimentalDrive(double speed, double distance, double timeoutS, boolean backgrnd) {
        int newTarget;

        //double initAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (Op.opModeIsActive()) {
            if (backgrnd) {
                speedG = speed;
                leftCMG = distance;
                timeoutSG = timeoutS;
                mode = 3;
                start();

            }
            newTarget = motorLeft.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newTarget);
            //motorRight.setTargetPosition(newRightTarget);
            Op.telemetry.addData("Encoder Target: ", "%7d", newTarget);
            Op.telemetry.addData("Current Position: ", "%7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            Op.telemetry.update();
            try {
                sleep(2000);
            } catch (InterruptedException e) {

            }
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            while (Op.opModeIsActive() && (runtime.seconds() < timeoutS) && (motorLeft.isBusy()))
            {
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                Op.telemetry.addData("Encoder Target: ", "%7d :%7d", newTarget);
                Op.telemetry.addData("Current Position: ", "%7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
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
            motorRight.setPower(0);
            motorLeft.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try {
                sleep(2000);
            } catch (InterruptedException e) {

            }

            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Op.sleep(250);
        }
    }
    void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS, boolean backgrnd) {
        int newLeftTarget;
        int newRightTarget;

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //double initAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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
            newRightTarget = motorRight.getCurrentPosition() - (int) (rightCM * COUNTS_PER_INCH);
            int lT = Math.abs(motorLeft.getCurrentPosition() - (int) (leftCM * COUNTS_PER_INCH));
            int rT = Math.abs(motorRight.getCurrentPosition() - (int) (rightCM * COUNTS_PER_INCH));
            Op.telemetry.addData("Encoder Target: ", "%7d :%7d", newLeftTarget, newRightTarget);
            Op.telemetry.addData("Current Position: ", "%7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            Op.telemetry.update();
            /*
            try {
                sleep(2000);
            } catch (InterruptedException e) {

            }
            */
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            runtime.reset();
            motorLeft.setPower(Math.copySign(Math.abs(speed),newLeftTarget));
            motorRight.setPower(Math.copySign(Math.abs(speed),newRightTarget));
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) { }

            while (Op.opModeIsActive() && (runtime.seconds() < timeoutS) && (Math.abs(motorLeft.getCurrentPosition()-newLeftTarget)>10
                    && Math.abs(motorRight.getCurrentPosition()-newRightTarget)>10)
                    && (lT+20 >= Math.abs(motorLeft.getCurrentPosition() - newLeftTarget)))
            {
                Op.telemetry.addData("Goodness:", "%7d, %7d",lT+20 - Math.abs(motorLeft.getCurrentPosition() - newLeftTarget), rT+20 - Math.abs(motorRight.getCurrentPosition() - newRightTarget));
                lT = Math.abs(motorLeft.getCurrentPosition() - newLeftTarget);
                rT = Math.abs(motorLeft.getCurrentPosition() - newRightTarget);
                if (Op.isStopRequested()) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                Op.telemetry.addData("Encoder Target: ", "%7d, %7d", newLeftTarget, newRightTarget);
                Op.telemetry.addData("Current Position: ", "%7d, %7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                Op.telemetry.addData("Special Numbers:", "%7d, %7d", lT, rT);
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
            Op.telemetry.addData("Goodness:", "%7d, %7d",lT+20 - Math.abs(motorLeft.getCurrentPosition() - newLeftTarget), rT+20 - Math.abs(motorRight.getCurrentPosition() - newRightTarget));
            Op.telemetry.addData("Encoder Target: ", "%7d, %7d", newLeftTarget, newRightTarget);
            Op.telemetry.addData("Current Position: ", "%7d, %7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            Op.telemetry.addData("Special Numbers:", "%7d, %7d", lT, rT);
            Op.telemetry.update();
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                sleep(250);
            } catch (InterruptedException e) {

            }

        }
    }
    void encoderDrive(double speed, double distance, double timeoutS) {
        encoderDrive(speed, distance, distance, timeoutS, false);
    }
    void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS) {
        encoderDrive(speed, leftCM, rightCM, timeoutS, false);
    }
    void experimentalDrive(double speed, double distance, double timeoutS) {
        experimentalDrive(speed, distance, timeoutS, false);
    }
    void angleTurn(double speed, double angle) {
        angleTurn(speed, angle, false);
    }
    public void run() {
        if (mode == 1) {
            angleTurn(speedG,angleG,false);
        } else if (mode == 2) {
            encoderDrive(speedG, leftCMG, rightCMG, timeoutSG,false);
        } else if (mode == 3) {
            experimentalDrive(speedG,leftCMG,timeoutSG,false);
        }
    }
}
