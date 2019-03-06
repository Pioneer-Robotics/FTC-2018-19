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

    void init(DcMotor motL, DcMotor motR, BNO055IMU im, LinearOpMode O, ElapsedTime run, double CPI) {
        //turns all the necessary robot parts into local variables as it is extremely tedious to have to write each as an argument for every individual function call.
        motorLeft = motL;
        motorRight = motR;
        imu = im;
        Op = O;
        runtime = run;
        COUNTS_PER_INCH = CPI;
    }
    void angleTurn(double speed, double angle, boolean abs, boolean backgrnd) {
        //Uses a PID algorithm
        double targetAngle; //Self-explanatory
        double time; //diagnostics, read how long each iteration of turn takes
        double maxtime = 0;//diagnostics, max acquisition time
        double maxdel = 0;//diagnostics, max delta (difference) of angles, angles/iteration
        //double start;
        double spd; //dynamic (can change) controller of turn speed
        double prp; //proportional error
        double itr = 0; //integral error
        double der; //differential error
        double pk = 7; //amplifier for proportion
        double ik = 0.1; //amplifier for integral
        double dk = 4; //amplifier for differential
        int direction; // -1 = cw, 1 = ccw. Determines the direction of the turn
        double dis; //distance to targetAngle
        double mspd; //max speed
        double prdis = 0; //previous distance
        int prdir = 0; //ed fcdfprevious direction
        if (Op.opModeIsActive()) {
            if (backgrnd) { //allows the program run in background as a separate task.
                angleG = angle;
                speedG = speed;
                mode = 1;
                start();
                return;
            }
            double mdis;
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //current angle of imu
            Orientation delta = angles; //delta of angle, essentially previous acquisition
            if (abs) targetAngle = angle;
            else targetAngle = angle + angles.firstAngle; //calculates target angle
            Op.telemetry.clearAll();
            if ((Math.abs((720-angles.firstAngle+targetAngle)%360))<(Math.abs((720-targetAngle+angles.firstAngle)%360))) { //calculates direction and distance to targetAngle
                mdis = (Math.abs((720-angles.firstAngle+targetAngle)%360));
                direction = -1;
            } else {
                mdis = (Math.abs((720-targetAngle+angles.firstAngle)%360));
                direction = 1;
            }
            mspd=mdis/angle;
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double margin = 0.2;
            runtime.reset();
            dis = mdis;
            prdir = direction;
            while (true) {
                //calculations for deltas and times for telemetry diagnostics

                time = runtime.milliseconds();
                if (time>maxtime) maxtime = time;
                if (Math.abs(angles.firstAngle-delta.firstAngle)> maxdel) maxdel = Math.abs(angles.firstAngle-delta.firstAngle);
                runtime.reset();
                if (Op.isStopRequested()) { //stops crashes of driver station
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                prdir = direction;
                prdis = dis;
                if ((Math.abs((720-angles.firstAngle+targetAngle)%360))<(Math.abs((720-targetAngle+angles.firstAngle)%360))) { //calculates direction and distance to targetAngle
                    direction = -1;
                    dis = (Math.abs((720-angles.firstAngle+targetAngle)%360));
                } else {
                    direction = 1;
                    dis = (Math.abs((720-targetAngle+angles.firstAngle)%360));
                }
                // Calculate speed from distance to targetAngle
                prp=dis/angle; //faster
                itr=itr+direction*dis*time;
                //der=(dis*direction-prdis*prdir)/time;
                der=(angles.firstAngle-delta.firstAngle)/time; // alternate derivative
                spd=pk*prp+ik*itr+dk*der;
                motorLeft.setPower(-direction*(/*Math.sqrt*/(Math.abs(speed*spd)+0.03))); //set motor power based on given speed against dynamic spd and sets direction appropriately
                motorRight.setPower(direction*(/*Math.sqrt*/(Math.abs(speed*spd)+0.03)));

                //actual telemetry for diagnostics
                Op.telemetry.addData("Error:", "%.5f", dis);
                Op.telemetry.addData("Max Speed:", "%.5f",mspd);
                Op.telemetry.addData("P:", "%.5f",prp);
                Op.telemetry.addData("I:", "%.5f",itr);
                Op.telemetry.addData("D:", "%.5f",der);
                Op.telemetry.addData("SPD:", "%.5f",spd);
                Op.telemetry.addData("Initial Distance:", "%.5f",mdis);
                Op.telemetry.addData("Left speed","%.5f",-direction*(Math.abs(speed*spd)));
                Op.telemetry.addData("Right speed","%.5f",direction*(Math.abs(speed*spd)));
                Op.telemetry.addData("Direction:", "%7d",direction);
                Op.telemetry.addData("Speed:", direction*Math.abs(speed*spd));
                Op.telemetry.addData("IMU Heading:", "%.5f", ((angles.firstAngle+360)%360));
                Op.telemetry.addData("target:", "%.5f", targetAngle);
                Op.telemetry.addData("time: ", "%.2f",time);
                Op.telemetry.addData("delta:", "%.2f", angles.firstAngle-delta.firstAngle);
                Op.telemetry.update();
                delta = angles; //slightly more calculations for the delta
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //acquires current angle
                if (dis < margin * speed) { //determines stop conditions, not in while loop condition because of bug with Java
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    break;
                }
            }
            //further telemetry to keep displaying values.
            Op.telemetry.addData("Finished", "!");
            Op.telemetry.addData("P:", "%.5f",prp);
            Op.telemetry.addData("I:", "%.5f",itr);
            Op.telemetry.addData("D:", "%.5f",der);
            Op.telemetry.addData("SPD:", "%.5f",spd);
            Op.telemetry.addData("Error:", "%.5f", dis);
            Op.telemetry.addData("Max Speed:", "%.5f",mspd);
            Op.telemetry.addData("Initial Distance:", "%.5f",mdis);
            Op.telemetry.addData("Left speed","%.5f",-direction*(Math.abs(speed*spd)));
            Op.telemetry.addData("Right speed","%.5f",direction*(Math.abs(speed*spd)));
            Op.telemetry.addData("Speed:", direction*Math.abs(speed*spd));
            Op.telemetry.addData("Margin:", "%.5f", margin * speed);
            Op.telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle);
            Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
            Op.telemetry.addData("target:", "%.5f", targetAngle);
            Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
            Op.telemetry.addData("time: ", "%.2f",time);
            Op.telemetry.addData("delta:", "%.2f", angles.firstAngle-delta.firstAngle);
            Op.telemetry.addData("maxtime:", "%.2f", maxtime);
            Op.telemetry.addData("maxdel:", "%.2f",maxdel);
            Op.telemetry.update();
            //reset motors for other uses.
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try {//wait to account for jitter, momentum, etc.
                sleep(100);
            } catch (InterruptedException ignored) {

            }
        }
    }

    void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS, boolean backgrnd) {
        //initialize target variables for encoderDrive
        int newLeftTarget;
        int newRightTarget;
        //reset motors, ensuring they are completely stopped while doing so.
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (Op.opModeIsActive()) {
            if (backgrnd) { //allows encoderDrive to run in the background
                speedG = speed;
                leftCMG = leftCM;
                rightCMG = rightCM;
                timeoutSG = timeoutS;
                mode = 2;
                start();
            }
            //calculates absolute point that we want to stop at.
            newLeftTarget = motorLeft.getCurrentPosition() - (int) (leftCM * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() - (int) (rightCM * COUNTS_PER_INCH);
            //calculates physical distance needed to be traveled.
            int lT = Math.abs(newLeftTarget);
            int rT = Math.abs(newRightTarget);
            //diagnostics to see how accurate the calculations are
            Op.telemetry.addData("Encoder Target: ", "%7d :%7d", newLeftTarget, newRightTarget);
            Op.telemetry.addData("Current Position: ", "%7d :%7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            Op.telemetry.update();
            //changes mode of the motor to run with encoder
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            runtime.reset();

            //sets speed for motors
            motorLeft.setPower(-Math.copySign(speed,leftCM));
            motorRight.setPower(-Math.copySign(speed, rightCM));

            //wait to prevent jitter
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) { }

            while (Op.opModeIsActive() && (runtime.seconds() < timeoutS) && (Math.abs(motorLeft.getCurrentPosition()-newLeftTarget)>2
                    && Math.abs(motorRight.getCurrentPosition()-newRightTarget)>2)
                    && (lT+10 >= Math.abs(motorLeft.getCurrentPosition() - newLeftTarget)))
            { //figures out when to stop. 1st condition: checks the current position of the robot versus the expected position of the robot.
                //If the distance is small, stop.
                //2nd condition: If the robot overshoots, then the first condition will stop being met, which means the robot will not stop.
                //To combat this problem, this condition checks whether the robot is getting closer to the target or not.
                //If the robot is getting farther away, then stop the robot.

                //calculations to read distance+telemetry to read distance
                Op.telemetry.addData("Goodness:", "%7d, %7d",lT+10 - Math.abs(motorLeft.getCurrentPosition() - newLeftTarget), rT+20 - Math.abs(motorRight.getCurrentPosition() - newRightTarget));
                lT = Math.abs(motorLeft.getCurrentPosition() - newLeftTarget);
                rT = Math.abs(motorLeft.getCurrentPosition() - newRightTarget);
                if (Op.isStopRequested()) { //prevents crashes when emergency stop is activated
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }

                //telemetry for diagnostics, reads exactly how encoderDrive is functioning.
                Op.telemetry.addData("Speeds:","%.5f, %.5f",Math.copySign(Math.abs(speed)*lT/((int) (leftCM * COUNTS_PER_INCH)),newLeftTarget),Math.copySign(Math.abs(speed)*rT/((int) (rightCM * COUNTS_PER_INCH)),newRightTarget));
                Op.telemetry.addData("Encoder Target: ", "%7d, %7d", newLeftTarget, newRightTarget);
                Op.telemetry.addData("Current Position: ", "%7d, %7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                Op.telemetry.addData("Distance from Target:", "%7d, %7d", lT, rT);
                Op.telemetry.update();
                if (Op.isStopRequested()) {//prevents crashes when emergency stop is activated
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
            }
            //telemetry for diagnostics, determines how well encoderDrive is working
            Op.telemetry.addData("Goodness:", "%7d, %7d",lT+20 - Math.abs(motorLeft.getCurrentPosition() - newLeftTarget), rT+20 - Math.abs(motorRight.getCurrentPosition() - newRightTarget));
            Op.telemetry.addData("Encoder Target: ", "%7d, %7d", newLeftTarget, newRightTarget);
            Op.telemetry.addData("Current Position: ", "%7d, %7d", motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
            Op.telemetry.addData("Special Numbers:", "%7d, %7d", lT, rT);
            Op.telemetry.update();
            //reset motors, ensuring they are completely stopped while doing so.
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //wait to prevent jitter
            try {
                sleep(250);
            } catch (InterruptedException ignored) {

            }

        }
    }
    //automatically removes the need for tank drive (leftCM and rightCM) and background parameters
    void encoderDrive(double speed, double distance, double timeoutS) {
        //if (experiment) experimentalDrive(speed,distance,timeoutS);
        encoderDrive(speed, distance, distance, timeoutS, false);
    }
    //automatically removes the need for background parameter
    void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS) {
        encoderDrive(speed, leftCM, rightCM, timeoutS, false);
    }
    //automatically removes the need for background parameter
    void angleTurn(double speed, double angle, boolean abs) {
        angleTurn(speed, angle, abs,false);
    }

    void angleTurn(double speed, double angle) {
        angleTurn(speed, angle, false);
    }

    //controls code running in background
    public void run() {
        if (mode == 1) {
            angleTurn(speedG,angleG);//check to see if this works! originally, was angleTurn, not experimental
        } else if (mode == 2) {
            encoderDrive(speedG, leftCMG, rightCMG, timeoutSG,false);
        }
    }
}