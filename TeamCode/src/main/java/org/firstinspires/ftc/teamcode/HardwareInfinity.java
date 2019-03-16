package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

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
    private LinearOpMode Op;
    WebcamName webCam;
    int tFlowId;
    private ElapsedTime runtime;
    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();
    private double speedG;
    private double angleG;
    private double leftCMG;
    private double rightCMG;
    private double timeoutSG;
    private int mode;
    double pk = 3; //gain for proportion
    double ik = 0.21; //gain for integral
    double dk = 0.39; //gain for differential

    //public Servo    rightClaw   = null;

    static final double lunchBoxMAX_POSITION = 0.6;
    static final double lunchBoxMIN_POSITION = 0;
    static final double LatchMAX_POSITION = 0;
    static final double LatchMIN_POSITION = 1;
    static final double DT_MIN = 0.5;
    static final double DT_MAX = 0.9;

    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareInfinity(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap, LinearOpMode aOp) {
        // Save reference to Hardware map
        /* local OpMode members. */
        runtime = new ElapsedTime();
        // Define and Initialize Motors
        Op = aOp;
        motorLeft  = ahwMap.get(DcMotor.class, "motorLeft");
        motorRight = ahwMap.get(DcMotor.class, "motorRight");
        linearArm  = ahwMap.get(DcMotor.class, "linearArm");
        armBase = ahwMap.get(DcMotor.class, "armBase");
        FBar = ahwMap.get(DcMotor.class, "fBar");
        Succq = ahwMap.get(DcMotor.class, "succq");
        linearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        linearArm.setDirection(DcMotor.Direction.REVERSE);
        FBar.setDirection(DcMotor.Direction.REVERSE);
        botSwitch = ahwMap.get(DigitalChannel.class, "botSwitch");
        topSwitch = ahwMap.get(DigitalChannel.class, "topSwitch");
        trigger = ahwMap.get(TouchSensor.class, "trigger");
        topSwitch.setMode(DigitalChannel.Mode.INPUT);
        botSwitch.setMode(DigitalChannel.Mode.INPUT);
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu1 = ahwMap.get(BNO055IMU.class, "imu1");
        webCam = ahwMap.get(WebcamName.class, "Webcam 1");
        tFlowId = ahwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
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
        lunchBox  = ahwMap.get(Servo.class, "lunchBox");
        Latch  = ahwMap.get(Servo.class, "Latch");
        Camera = ahwMap.get(Servo.class, "Camera");
        dropTop = ahwMap.get(Servo.class, "dropTop");

        lunchBox.setPosition(lunchBoxMAX_POSITION);
        Latch.setPosition(LatchMAX_POSITION);
        Camera.setPosition(0.5);
        dropTop.setPosition(DT_MAX);

    }
    void init(HardwareMap ahwMap) {
        init(ahwMap, null);
    }

    void angleTurn(double speed, double angle, boolean abs, boolean backgrnd) {
        if (Op==null) return;
        //Uses compList PID algorithm to turn accurately on point
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
        double prdis; //previous distance
        int prdir; //ed fcdfprevious direction
        ArrayList<Integer> prdi = new ArrayList<>();
        int yeet = 0; //yeet counter
        double im1;
        double im2;
        if (Op.opModeIsActive()) {
            if (backgrnd) { //allows the program run in background as compList separate task.
                angleG = angle;
                speedG = speed;
                mode = 1;
                start();
                return;
            }
            double mdis;
            im1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            im2 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (im1 - im2 > 180) im2 += 360;
            else if (im2 - im1 > 180) im1 += 360;
            double angl = ((im1+im2)/2)%360; //acquires current angle //current angle of imu
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
            while (true) {
                //Calculations for deltas and times for telemetry diagnostics

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
                prdi.add(direction);
                if (prdi.size() >= 10) prdi.remove(0);
                prdis = dis;
                if ((Math.abs((720-angl+targetAngle)%360))<(Math.abs((720-targetAngle+angl)%360))) { //calculates direction and distance to targetAngle
                    direction = -1;
                    dis = (Math.abs((720-angl+targetAngle)%360));
                } else {
                    direction = 1;
                    dis = (Math.abs((720-targetAngle+angl)%360));
                }
                //Calculate PID values from the error of distance/angle
                prp=dis/angle;
                itr=itr+direction*(dis/angle)*time/1000;
                der=((dis*direction/angle)-(prdis*prdir/angle))*1000/time;
                //Calculate speed from distance to targetAngle
                spd=pk*prp+ik*itr+dk*der;
                motorLeft.setPower(-direction*(/*Math.sqrt*/(Math.abs(speed*spd)+0.03))); //set motor power based on given speed against dynamic spd and sets direction appropriately
                motorRight.setPower(direction*(/*Math.sqrt*/(Math.abs(speed*spd)+0.03)));
                //counting the yeets
                boolean inc = false;
                for (int i = 0; i<prdi.size()-1;i++) {
                    if (prdi.get(i) == -prdi.get(i+1)) inc = true;
                }
                if (inc) yeet += 1;
                else yeet = 0;
                //actual telemetry for diagnostics
                Op.telemetry.addData("Error:", "%.5f", dis);
                Op.telemetry.addData("Yeet Counter:", "%d", yeet);
                Op.telemetry.addData("7:","%d",prdi.size());
                //Op.telemetry.addData("Max Speed:", "%.5f",mspd);
                //Op.telemetry.addData("P:", "%.5f",prp);
                //Op.telemetry.addData("I:", "%.5f",itr);
                //Op.telemetry.addData("D:", "%.5f",der);
                //Op.telemetry.addData("SPD:", "%.5f",spd);
                //Op.telemetry.addData("Initial Distance:", "%.5f",mdis);
                //Op.telemetry.addData("Left speed","%.5f",-direction*(Math.abs(speed*spd)));
                //Op.telemetry.addData("Right speed","%.5f",direction*(Math.abs(speed*spd)));
                //Op.telemetry.addData("Direction:", "%7d",direction);
                //Op.telemetry.addData("Speed:", direction*Math.abs(speed*spd));
                //Op.telemetry.addData("IMU Heading:", "%.5f", ((angl+720)%360));
                //Op.telemetry.addData("target:", "%.5f", targetAngle);
                //Op.telemetry.addData("time: ", "%.2f",time);
                //Op.telemetry.addData("delta:", "%.2f", angl-delta);
                Op.telemetry.update();
                delta = angl; //slightly more calculations for the delta
                im1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                im2 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (im1 - im2 > 180) im2 += 360;
                else if (im2 - im1 > 180) im1 += 360;
                angl = ((im1+im2)/2)%360; //acquires current angle
                if (dis < margin || yeet >= 10) { //determines stop conditions, not in while loop condition because of bug with Java
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    break;
                }
            }
            if (dis > 5) {
                Op.telemetry.addData("Take 2",1);
                Op.telemetry.update();
                try {//wait to account for jitter, momentum, etc.
                    sleep(500);
                } catch (InterruptedException ignored) {}
                angleTurn(0.3, targetAngle,true);
                return;
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
            } catch (InterruptedException ignored) {}
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
            } catch (InterruptedException ignored) {}

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
            angleTurn(speedG,angleG);
        } else if (mode == 2) {
            encoderDrive(speedG, leftCMG, rightCMG, timeoutSG,false);
        }
    }
}