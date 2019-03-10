package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous (name="TMAuto", group="FTCPio")
public class TMAuto extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;

    // State used for updating telemetry
    private int choose;


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Acceleration gravity = imu.getGravity();

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);

        CVManager tFlow = new CVManager();
        CamManager camM = new CamManager();
        Movement mov = new Movement();
        tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()),camM);
        camM.init(robot, tFlow);
        mov.init(robot.motorLeft,robot.motorRight,robot.imu, robot.imu1, runtime, COUNTS_PER_INCH, this);
        camM.reference = angles.firstAngle;
        tFlow.disable = true;
        camM.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        tFlow.start();

        telemetry.addData("Path0", "Starting at %7d:%7d",
                robot.motorLeft.getCurrentPosition(), robot.motorRight.getCurrentPosition());
        telemetry.update();

        //Drop down off lander - lowering robot


        /*     ACTUAL MOVEMENT--------------------------------------------------------------*/


        while (robot.linearArm.getCurrentPosition()<= 13516)  /* Most effective detachment point might not be at the top*/ {
            //positive = up
            telemetry.addData("Status: ", "Lowering robot");
            telemetry.addData("Encoder: ", robot.linearArm.getCurrentPosition());
            telemetry.addData("Bottom is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.Status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.update();
            robot.linearArm.setPower(1);
            if (robot.topSwitch.getState()) {
                robot.linearArm.setPower(0);
                break;
            }
            if (this.isStopRequested()) {
                robot.linearArm.setPower(0);
                camM.go = false;
                tFlow.go =false;
                return;
            }

        }
        robot.linearArm.setPower(0);

        // Detach from lander
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        telemetry.addData("Latches", "Min");
        telemetry.addData("Status: ", "Disengaging From Lander");
        telemetry.update();
        runtime.reset();
        while (!robot.botSwitch.getState() && !robot.topSwitch.getState() && runtime.milliseconds() < 3000) {
            robot.linearArm.setPower(1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.Status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.update();

        }
        robot.linearArm.setPower(0);
        /*
        runtime.reset();
        while (tFlow.go && runtime.milliseconds() <= 10000) {
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.Status);
            telemetry.addData("MineralX:","%.5f",tFlow.mineralX);
            telemetry.update();
            sleep(1);
        }
        */
        sleep(100);
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.Status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();

        //Drive away
        mov.encoderDrive(0.5,5,10);

        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.Status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();
        runtime.reset();
        while (tFlow.go && runtime.milliseconds() <= 2000) {
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.Status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.update();
            sleep(1);
        }

        // Tensorflow output stored into variable to choose where robot should go
        choose = (int) tFlow.minDat[0];
        camM.go = false;
        tFlow.go = false;
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.Status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();

        // Perform sampling and position for dropping team marker
        //sleep(6000);
        switch (choose) {
            case 1:
                //Mineral on Left
                mov.angleTurn(TURN_SPEED,36);
                mov.encoderDrive(DRIVE_SPEED,31, 5);
                mov.angleTurn(TURN_SPEED, -86);
                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                mov.encoderDrive(DRIVE_SPEED,10, 5);
                //mov.angleTurn(TURN_SPEED, 85);

                break;
            case 2:
                //Mineral in Middle
                //no turning movement is necessary to hit mineral
                mov.encoderDrive(DRIVE_SPEED,38, 5);

                break;
            case 3:
                //Mineral on Right
                mov.angleTurn(TURN_SPEED,-38);
                mov.encoderDrive(DRIVE_SPEED,33, 5);
                mov.angleTurn(TURN_SPEED, 70);
                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                mov.encoderDrive(DRIVE_SPEED,5,5);
                //mov.angleTurn(TURN_SPEED, 30);

                //mov.angleTurn(0.2, 90, false);
                //mov.encoderDrive( 0.5, 5,5,10, false);
                break;
            default:
                //error happened with TensorFlow
                telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                // if tensor flow doesn't function, the robot will default to moving to the middle position
                mov.encoderDrive(DRIVE_SPEED,13,5);
                mov.encoderDrive(DRIVE_SPEED,10, 5);

                mov.angleTurn(TURN_SPEED, 90);
                sleep(2000);
                break;
        }
        telemetry.addData("Status: ", "Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
        sleep(500);
        telemetry.addData("Status: ", "Dropped Team Marker");
        telemetry.update();
        //sleep(1500);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        telemetry.update();

        // Different maneuvering to get to crater depending on which position sampling mineral was in
        // Goal is to line up with wall after team marker dropped

        switch (choose) {
            case 1:
                mov.angleTurn(TURN_SPEED, -35);
                mov.encoderDrive(DRIVE_SPEED,17, 5);
                mov.angleTurn(TURN_SPEED, -32);
                mov.encoderDrive(1,52, 10);
                break;
            case 2:
                mov.angleTurn(TURN_SPEED, 72);
                mov.encoderDrive(DRIVE_SPEED,-21, 5);
                mov.angleTurn(TURN_SPEED, -34);
                mov.encoderDrive(1,-52, 10);
                break;
            case 3:
                mov.encoderDrive(DRIVE_SPEED,-21, 5);
                mov.angleTurn(TURN_SPEED, -35);
                mov.encoderDrive(1,-52, 10);
                break;
        }
        // Back up into crater at max speed

        robot.Camera.setPosition(0);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        while (!robot.botSwitch.getState()) {
            robot.linearArm.setPower(-1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.update();
            if (this.isStopRequested()) {
                robot.linearArm.setPower(0);
                break;
            }

        }
        robot.linearArm.setPower(0);
        //encoderDrive(1, 255,255,30);


        //Drop Team Marker

        /*while (opModeIsActive())
        {
            telemetry.addData("Power L", robot.motorLeft.getPower());
            telemetry.addData("Power R", robot.motorRight.getPower());
            telemetry.addData("Rotations L", robot.motorLeft.getCurrentPosition() / TETRIX_TICKS_PER_REV);
            telemetry.addData("Rotations R", robot.motorLeft.getCurrentPosition() / TETRIX_TICKS_PER_REV);
            telemetry.addData("Linear Encoder", robot.linearArm.getCurrentPosition());
            telemetry.update();
        }*/
        //Tank(0,0);
        telemetry.addData("Status: ", "Finished");
        telemetry.update();
    }
}
