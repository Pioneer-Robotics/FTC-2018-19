package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import java.util.Locale;


@Autonomous (name="LL5156: Auto Drive by Encoder", group="LL5156")
public class SimpleAuto extends LinearOpMode {
    HardwareLL5156 robot = new HardwareLL5156();
    private ElapsedTime runtime = new ElapsedTime();
    //private Gyroscope imu;
    //private DcMotor motorLeft;
    //private DcMotor motorLeft;

    static final double TETRIX_TICKS_PER_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;


    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".


    /*public double Rotations(double Rotation)
    {
        return Rotation*TETRIX_TICKS_PER_REV;
    }

    public void Tank(double powerL,double powerR)
    {
        motorRight.setPower(powerR);
        motorLeft.setPower(powerL);
    }*/
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();
        robot.init(hardwareMap);
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        IParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IParameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TensorFlowSource tFlow = new TensorFlowSource();
        tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        tFlow.start();
        camVision(angles.firstAngle);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Tank(1,1);
        //(motorLeft.getCurrentPosition() < Rotations(2))

        telemetry.addData("Path0", "Starting at %7d:%7d",
                robot.motorLeft.getCurrentPosition(), robot.motorRight.getCurrentPosition());
        telemetry.update();


        //ACTUAL MOVEMENT

        //Drop down off lander - lowering robot
        while (!robot.topSwitch.getState())  /* Most effective detachment point might not be at the top*/ {
            //positive = up
            telemetry.addData("Status: ", "Lowering robot");
            telemetry.update();
            robot.linearArm.setPower(1);

        }
        robot.linearArm.setPower(0);

        // Detach from lander
        robot.Latch.setPosition(HardwareLL5156.LatchMAX_POSITION);
        telemetry.addData("Latches", "Max");
        telemetry.addData("Status: ", "Disengaging From Lander");
        telemetry.update();

        //Drive away
        encoderDrive(DRIVE_SPEED, -10, -10, 5.0);

        //Run Google Tensor Flow to detect object.....
        telemetry.addData("Status: ", "Detecting Gold Sample");
        telemetry.update();

        if (tFlow.Status == 1)
        {
            //left
            robot.motorLeft.setPower(0.75);
            robot.motorRight.setPower(-0.75);

            while (!(angles.firstAngle <= 45 /* add angular # */))
            {
                robot.motorLeft.setPower(0);
                robot.motorRight.setPower(0);
            }
        }
        formatAngle(angles.angleUnit, angles.firstAngle);

        encoderDrive(TURN_SPEED, 16, -16, 5.0);
        //NEED TO TEST MORE, (16,-16) is close to 90 degrees
        //encoderDrive(DRIVE_SPEED,24,24,4.0);
        sleep(250);

        //Drop Team Marker
        telemetry.addData("Status: ", "Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMIN_POSITION);
        telemetry.addData("Status: ", "Dropped Team Marker");
        telemetry.update();
        sleep(500);
        robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMAX_POSITION);


        while (opModeIsActive())
        {
            telemetry.addData("Power L", robot.motorLeft.getPower());
            telemetry.addData("Power R", robot.motorRight.getPower());
            telemetry.addData("Rotations L", robot.motorLeft.getCurrentPosition() / TETRIX_TICKS_PER_REV);
            telemetry.addData("Rotations R", robot.motorLeft.getCurrentPosition() / TETRIX_TICKS_PER_REV);
            telemetry.addData("Linear Encoder", robot.linearArm.getCurrentPosition());
            telemetry.update();
        }
        //Tank(0,0);
        telemetry.addData("Status: ", "Finished");
        telemetry.update();
    }
    public void angleTurn(double speed, double angle) {
        double targetAngle;
        if (opModeIsActive()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetAngle = angle+angles.firstAngle;
            while (angles.firstAngle+180 > (targetAngle+180+50*speed)%360 || angles.firstAngle < (targetAngle+180-50*speed)%360) {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (angles.firstAngle+180 > targetAngle%360) {
                    robot.motorLeft.setPower(Math.abs(speed));
                    robot.motorRight.setPower(-Math.abs(speed));
                } else {
                    robot.motorLeft.setPower(-Math.abs(speed));
                    robot.motorRight.setPower(Math.abs(speed));
                }
                telemetry.addData("IMU Heading:", "%.5f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+180);
                telemetry.addData("target:","%.5f",targetAngle+180);
                telemetry.update();
            }
        }
    }
    public void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        double initAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (opModeIsActive()) {
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int) (leftCM * COUNTS_PER_INCH);
            newRightTarget = robot.motorLeft.getCurrentPosition() + (int) (rightCM * COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed));
            robot.motorRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy()))
            {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > initAng+0.01
                        || imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle < initAng-0.01) {

                    int mLt = robot.motorLeft.getCurrentPosition();
                    int mRt = robot.motorRight.getCurrentPosition();
                    angleTurn(1, initAng);
                    robot.motorLeft.setTargetPosition(newLeftTarget+(robot.motorLeft.getCurrentPosition()-mLt));
                    robot.motorRight.setTargetPosition(newRightTarget+(robot.motorRight.getCurrentPosition()-mRt));
                }
                telemetry.update();
            }

            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
    //a calibration of the imu needs to be put at the start of the Simple Auto
    /*NOTE by Jeremy 11/30/18: This should work, if the rest of Michael's code is good. What I did was set the calibration angle
    angleZero as the input in a final float format, so it can't change. We always input with angles.firstAngle.
    Then, angles.firstAngle is again repeatedly sensed, but since angleZero is a final value, it should be a reference point.
    Thus, the camera should move as intended, provided the camera movement code is correct.*/
    public void camVision(final float angleZero)
    {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle > (angleZero - 90) && angles.firstAngle < (angleZero + 90) )
        {
            robot.Camera.setPosition((Math.abs(angles.firstAngle * (0.5/90)))   -    0.5);
            //robot is turned to the left, cam stays center with objects
        }
        else
        {
            robot.Camera.setPosition(0.5);
            //robot is past 90 degrees in either direction, cam moves to center
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}