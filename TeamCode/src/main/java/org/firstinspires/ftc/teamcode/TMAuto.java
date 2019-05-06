package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous (name="TMAuto", group="FTCPio")
public class TMAuto extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;

    // State used for updating telemetry
    private int choose;

    CVManager tFlow = new CVManager();
    CamManager camM = new CamManager();


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on compList Core Device Interface Module, configured to be compList sensor of type "AdaFruit IMU",
    // and named "imu".



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
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

        tFlow.init(robot.webCam, robot.tFlowId, camM);
        camM.init(robot, tFlow);
        camM.reference = angles.firstAngle;
        tFlow.autoDisable = true;
        camM.start();

        telemetry.addData("status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        tFlow.start();

        telemetry.addData("Path0", "Starting at %7d:%7d",
                robot.motorLeft.getCurrentPosition(), robot.motorRight.getCurrentPosition());
        telemetry.update();

        //Drop down off lander - lowering robot


        /*     ACTUAL MOVEMENT--------------------------------------------------------------*/


        while (robot.linearArm.getCurrentPosition()<= 4700)  /* Most effective detachment point might not be at the top*/ {
            //positive = up
            telemetry.addData("status: ", "Lowering robot");
            telemetry.addData("Encoder: ", robot.linearArm.getCurrentPosition());
            telemetry.addData("Bottom is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("status:","%d",tFlow.status);
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
        telemetry.addData("status: ", "Disengaging From Lander");
        telemetry.update();
        runtime.reset();
        while (!robot.botSwitch.getState() && !robot.topSwitch.getState() && runtime.milliseconds() < 3000) {
            robot.linearArm.setPower(1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("status:","%d",tFlow.status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.update();

        }
        robot.linearArm.setPower(0);
        /*
        runtime.reset();
        while (tFlow.go && runtime.milliseconds() <= 10000) {
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("status:","%d",tFlow.status);
            telemetry.addData("MineralX:","%.5f",tFlow.mineralX);
            telemetry.update();
            sleep(1);
        }
        */
        sleep(100);
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();

        //Drive away
        robot.encoderDrive(0.5,5,10);

        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();
        runtime.reset();
        while (tFlow.go && runtime.milliseconds() <= 2000) {
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("status:","%d",tFlow.status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.update();
            sleep(1);
        }

        // Tensorflow output stored into variable to choose where robot should go
        choose = (int) tFlow.minDat[0];
        camM.go = false;
        tFlow.go = false;
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();

        // Perform sampling and position for dropping team marker
        //sleep(6000);
        switch (choose) {
            case 1:
                //Mineral on Left
                robot.angleTurn(TURN_SPEED,33, true);
                robot.encoderDrive(DRIVE_SPEED,31, 5);
                robot.angleTurn(0.3, -83);
                //telemetry.addData("TFlow says: ", "%d",tFlow.status);
                robot.encoderDrive(DRIVE_SPEED,13, 5);

                break;
            case 2:
                //Mineral in Middle
                //no turning robotement is necessary to hit mineral
                robot.encoderDrive(DRIVE_SPEED,38, 5);

                break;
            case 3:
                //Mineral on Right
                robot.angleTurn(TURN_SPEED,-38);
                robot.encoderDrive(DRIVE_SPEED,33, 5);
                robot.angleTurn(0.3, 59);
                //telemetry.addData("TFlow says: ", "%d",tFlow.status);
                robot.encoderDrive(DRIVE_SPEED,7,5);

                break;
            default:
                //error happened with TensorFlow
                telemetry.addData("TFlow says: ", "%d",tFlow.status);
                // if tensor flow doesn't function, the robot will default to roboting to the middle position
                robot.encoderDrive(DRIVE_SPEED,38, 5);

                break;
        }
        telemetry.addData("status: ", "Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
        sleep(500);
        telemetry.addData("status: ", "Dropped Team Marker");
        telemetry.update();
        //sleep(1500);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        telemetry.update();

        // Different maneuvering to get to crater depending on which position sampling mineral was in
        // Goal is to line up with wall after team marker dropped

        switch (choose) {
            case 1:
                robot.angleTurn(0.3, -36);
                robot.encoderDrive(DRIVE_SPEED,19, 5);
                robot.angleTurn(TURN_SPEED, -47);
                robot.encoderDrive(1,52, 10);
                break;
            case 2:
                robot.angleTurn(TURN_SPEED, 72);
                robot.encoderDrive(DRIVE_SPEED,-23, 5);
                robot.angleTurn(TURN_SPEED, -31, true);
                robot.encoderDrive(1,-55, 10);
                break;
            case 3:
                robot.angleTurn(0.1, 30, true);
                robot.encoderDrive(1,-55, 10);
                break;
            default:
                robot.angleTurn(TURN_SPEED, 72);
                robot.encoderDrive(DRIVE_SPEED,-23, 5);
                robot.angleTurn(TURN_SPEED, -31, true);
                robot.encoderDrive(1,-55, 10);
                break;
        }
        // Back up into crater at max speed
        if (this.isStopRequested()) {
            robot.linearArm.setPower(0);
            return;
        }
        robot.Camera.setPosition(0);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        /*
        while (!robot.botSwitch.getState()) {
            robot.linearArm.setPower(-1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.update();
            if (this.isStopRequested()) {
                robot.linearArm.setPower(0);
                return;
            }

        }*/
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
        telemetry.addData("status: ", "Finished");
        telemetry.update();
    }
}
