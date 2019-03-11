package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Motor channel:  Left  drive motor:        "motorLeft"
 * Motor channel:  Right drive motor:        "motorRight"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
class HardwareInfinity extends Thread
{
    /* Public OpMode members. */
    DcMotor  motorLeft;
    DcMotor  motorRight;
    DcMotor  linearArm;
    DcMotor armBase;
    DcMotor FBar;
    DcMotor Succq;
    Servo lunchBox;
    Servo Latch;
    Servo Camera;
    Servo dropTop;
    DigitalChannel botSwitch;
    DigitalChannel topSwitch;
    TouchSensor trigger;
    BNO055IMU imu;
    BNO055IMU imu1;
    LinearOpMode Op;
    private ElapsedTime runtime;
    private double COUNTS_PER_INCH;
    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();
    private double speedG;
    private double angleG;
    private double leftCMG;
    private double rightCMG;
    private double timeoutSG;
    private int mode;
    double pk = 3; //gain for proportion
    double ik = 0.2; //gain for integral
    double dk = 0.3; //gain for differential

    //public Servo    rightClaw   = null;

    static final double lunchBoxMAX_POSITION = 0.6;
    static final double lunchBoxMIN_POSITION = 0;
    static final double LatchMAX_POSITION = 0;
    static final double LatchMIN_POSITION = 1;
    static final double DT_MIN = 0.5;
    static final double DT_MAX = 0.9;

    /* local OpMode members. */
    private HardwareMap hwMap;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareInfinity(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap, LinearOpMode aOp, ElapsedTime run, double CPI) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        runtime = run;
        COUNTS_PER_INCH = CPI;
        // Define and Initialize Motors
        Op = aOp;
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        linearArm  = hwMap.get(DcMotor.class, "linearArm");
        armBase = hwMap.get(DcMotor.class, "armBase");
        FBar = hwMap.get(DcMotor.class, "fBar");
        Succq = hwMap.get(DcMotor.class, "succq");
        linearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        FBar.setDirection(DcMotor.Direction.REVERSE);
        botSwitch = hwMap.get(DigitalChannel.class, "botSwitch");
        topSwitch = hwMap.get(DigitalChannel.class, "topSwitch");
        trigger = hwMap.get(TouchSensor.class, "trigger");
        topSwitch.setMode(DigitalChannel.Mode.INPUT);
        botSwitch.setMode(DigitalChannel.Mode.INPUT);
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu1 = hwMap.get(BNO055IMU.class, "imu1");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        imu.initialize(IParameters);
        IParameters.loggingTag = "IMU1";
        imu1.initialize(IParameters);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        linearArm.setPower(0);
        armBase.setPower(0);
        FBar.setPower(0);

        // Set motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Succq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        lunchBox  = hwMap.get(Servo.class, "lunchBox");
        Latch  = hwMap.get(Servo.class, "Latch");
        Camera = hwMap.get(Servo.class, "Camera");
        dropTop = hwMap.get(Servo.class, "dropTop");

        lunchBox.setPosition(lunchBoxMAX_POSITION);
        Latch.setPosition(LatchMAX_POSITION);
        Camera.setPosition(0.5);
        dropTop.setPosition(DT_MAX);

    }
    void angleTurn(double speed, double angle, boolean abs, boolean backgrnd) {
        if (Op==null) return;
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
            double angl = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)/2; //current angle of imu
            double delta = angl; //delta of angle, essentially previous acquisition
            if (abs) targetAngle = angle;
            else targetAngle = angle + angl; //calculates target angle
            Op.telemetry.clearAll();
            if ((Math.abs((720-angl+targetAngle)%360))<(Math.abs((720-targetAngle+angl)%360))) { //calculates direction and distance to targetAngle
                mdis = (Math.abs((720-angl+targetAngle)%360));
                direction = -1;
            } else {
                mdis = (Math.abs((720-targetAngle+angl)%360));
                direction = 1;
            }
            mspd=mdis/angle;
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double margin = 0.15;
            runtime.reset();
            dis = mdis;
            prdir = direction;
            while (true) {
                //calculations for deltas and times for telemetry diagnostics

                time = runtime.milliseconds();
                if (time>maxtime) maxtime = time;
                if (Math.abs(angl-delta)> maxdel) maxdel = Math.abs(angl-delta);
                runtime.reset();
                if (Op.isStopRequested()) { //stops crashes of driver station
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    return;
                }
                prdir = direction;
                prdis = dis;
                if ((Math.abs((720-angl+targetAngle)%360))<(Math.abs((720-targetAngle+angl)%360))) { //calculates direction and distance to targetAngle
                    direction = -1;
                    dis = (Math.abs((720-angl+targetAngle)%360));
                } else {
                    direction = 1;
                    dis = (Math.abs((720-targetAngle+angl)%360));
                }
                // Calculate speed from distance to targetAngle
                prp=dis/angle; //faster
                itr=itr+direction*(dis/angle)*time/1000;
                //der=(dis*direction-prdis*prdir)/time;
                der=((dis*direction/angle)-(prdis*prdir/angle))*1000/time; // alternate derivative
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
                //Op.telemetry.addData("Initial Distance:", "%.5f",mdis);
                //Op.telemetry.addData("Left speed","%.5f",-direction*(Math.abs(speed*spd)));
                //Op.telemetry.addData("Right speed","%.5f",direction*(Math.abs(speed*spd)));
                //Op.telemetry.addData("Direction:", "%7d",direction);
                Op.telemetry.addData("Speed:", direction*Math.abs(speed*spd));
                Op.telemetry.addData("IMU Heading:", "%.5f", ((angl+720)%360));
                //Op.telemetry.addData("target:", "%.5f", targetAngle);
                Op.telemetry.addData("time: ", "%.2f",time);
                Op.telemetry.addData("delta:", "%.2f", angl-delta);
                Op.telemetry.update();
                delta = angl; //slightly more calculations for the delta
                angl = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)/2; //acquires current angle
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
            Op.telemetry.addData("IMU Heading:", "%.5f", angl);
            Op.telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
            Op.telemetry.addData("target:", "%.5f", targetAngle);
            Op.telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
            Op.telemetry.addData("time: ", "%.2f",time);
            Op.telemetry.addData("delta:", "%.2f", angl-delta);
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
        if (Op==null) return;
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
        if (Op==null) return;
        //if (experiment) experimentalDrive(speed,distance,timeoutS);
        encoderDrive(speed, distance, distance, timeoutS, false);
    }
    //automatically removes the need for background parameter
    void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS) {
        if (Op==null) return;
        encoderDrive(speed, leftCM, rightCM, timeoutS, false);
    }
    //automatically removes the need for background parameter
    void angleTurn(double speed, double angle, boolean abs) {
        if (Op==null) return;
        angleTurn(speed, angle, abs,false);
    }

    void angleTurn(double speed, double angle) {
        if (Op==null) return;
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