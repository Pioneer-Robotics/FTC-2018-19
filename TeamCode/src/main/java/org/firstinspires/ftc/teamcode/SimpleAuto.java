package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import java.util.Locale;


@Autonomous (name="TensorAuto", group="FTCPio")
public class SimpleAuto extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;

    // State used for updating telemetry
    private int choose;


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
        robot.init(hardwareMap);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Acceleration gravity = imu.getGravity();

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CVManager tFlow = new CVManager();
        CamManager camM = new CamManager();
        tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        camM.init(robot.imu,robot);
        camM.reference = angles.firstAngle;
        camM.start();
        tFlow.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Tank(1,1);
        //(motorLeft.getCurrentPosition() < Rotations(2))

        telemetry.addData("Path0", "Starting at %7d:%7d",
                robot.motorLeft.getCurrentPosition(), robot.motorRight.getCurrentPosition());
        telemetry.update();

        // x tFlow.extractPos(tFlow.location)[0]
        // y tFlow.extractPos(tFlow.location)[1]
        //Drop down off lander - lowering robot



        /*     ACTUAL MOVEMENT--------------------------------------------------------------*/


        while (robot.linearArm.getCurrentPosition()<= 14916)  /* Most effective detachment point might not be at the top*/ {
            //positive = up
            telemetry.addData("Status: ", "Lowering robot");
            telemetry.update();
            robot.linearArm.setPower(1);

        }
        robot.linearArm.setPower(0);

        // Detach from lander
        robot.Latch.setPosition(HardwareInfinity.LatchMAX_POSITION);
        telemetry.addData("Latches", "Max");
        telemetry.addData("Status: ", "Disengaging From Lander");
        telemetry.update();

        //Drive away
        encoderDrive(0.5,35,35,10);
        int choose = tFlow.Status;
        if (tFlow.Status == -3) {
            if (tFlow.mineralX<233) {
                choose = 1;
            } else if (tFlow.mineralX<466) {
                choose = 2;
            } else if (tFlow.mineralX!=0) {
                choose = 3;
            } else {
                choose = -4;
            }
        }

        switch (choose) {
            case 1:
                //left
                angleTurn(0.5,20);
                        /*robot.motorLeft.setPower(0.75);
                        robot.motorRight.setPower(-0.75);

                        while (!(angles.firstAngle <= 38.8))
                        {
                            robot.motorLeft.setPower(0.75);
                            robot.motorRight.setPower(-0.75);
                        }
                        robot.motorLeft.setPower(0);
                        robot.motorRight.setPower(0);*/
                encoderDrive(DRIVE_SPEED, 61.51, 61.51, 5);
                angleTurn(0.5, -20);
                telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                encoderDrive( 0.5,10,10,10);

                angleTurn(0.3, 90);
                break;
            case 2:
                //middle
                //theoretically no movement is necessary
                telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                encoderDrive(DRIVE_SPEED, 49.26, 49.26, 5);
                encoderDrive( 0.5,10,10,10);

                angleTurn(0.3, 90);
                encoderDrive( 0.5, 10,10,10);
                break;
            case 3:
                //right
                angleTurn(0.5,-20);
                        /*robot.motorLeft.setPower(-0.75);
                        robot.motorRight.setPower(0.75);

                        while (!(angles.firstAngle >= -36.8))
                        {
                            robot.motorLeft.setPower(-0.75);
                            robot.motorRight.setPower(0.75);

                        }
                        robot.motorLeft.setPower(0);
                        robot.motorRight.setPower(0);*/
                encoderDrive(DRIVE_SPEED, 61.51, 61.51, 5);
                angleTurn(0.5, 20);
                telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                encoderDrive( 0.5,10,10,10);
                angleTurn(0.3, 90);
                encoderDrive( 0.5, 20,20,10);
                break;
            case -3:
                //this is the manual mode, shouldn't ever be used
                telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                telemetry.addData("TFlow says: ", "%.5f",tFlow.mineralX);
                telemetry.update();
                break;
            default:
                //error happened with TensorFlow
                telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                // if tensor flow doesn't function, the robot will default to moving to the middle position
                encoderDrive(DRIVE_SPEED, 13, 13, 5);
                encoderDrive( 0.5,10,10,10);

                angleTurn(0.3, 90);
                break;
        }
        telemetry.update();


        //encoderDrive(TURN_SPEED, 16, -16, 5.0);
        //angleTurn(0.5,-90);
        //NEED TO TEST MORE, (16,-16) is close to 90 degrees
        //encoderDrive(DRIVE_SPEED,1000,1000,4.0);
        telemetry.addData("Status: ", "Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
        telemetry.addData("Status: ", "Dropped Team Marker");
        telemetry.update();
        sleep(500);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        
        angleTurn(0.3, 45);
        encoderDrive(1, 305,305,30);
        while (!robot.botSwitch.getState()) {
            robot.linearArm.setPower(-1);
        }
        robot.linearArm.setPower(0);

        //Drop Team Marker

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
        tFlow.go = false;
        camM.go = false;
    }
    private void angleTurn(double speed, double angle) {
        double targetAngle;
        int margin = 7;
        if (opModeIsActive()) {
            Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetAngle = angle+angles.firstAngle+180;
            telemetry.clearAll();
            while (Math.abs(angles.firstAngle+180-targetAngle) > margin*speed && (360-Math.abs(angles.firstAngle+180-targetAngle)) > margin*speed) {
                if (angle>0) {
                    robot.motorLeft.setPower(-Math.abs(speed));
                    robot.motorRight.setPower(Math.abs(speed));
                } else {
                    robot.motorLeft.setPower(Math.abs(speed));
                    robot.motorRight.setPower(-Math.abs(speed));
                }
                telemetry.addData("Error:","%.5f", Math.abs(angles.firstAngle+180-targetAngle));
                telemetry.addData("Margin:","%.5f",margin*speed);
                telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
                telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
                telemetry.addData("target:", "%.5f", targetAngle);
                telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
                telemetry.update();
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);
        }
    }
    private void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        double initAng = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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
                telemetry.addData("Path2", "Running at %7d :%7d", robot.motorLeft.getCurrentPosition(), robot.motorRight.getCurrentPosition());
                if (robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > initAng+10
                        || robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle < initAng-10) {

                    int mLt = robot.motorLeft.getCurrentPosition();
                    int mRt = robot.motorRight.getCurrentPosition();
                    angleTurn(0.5, initAng);
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

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
