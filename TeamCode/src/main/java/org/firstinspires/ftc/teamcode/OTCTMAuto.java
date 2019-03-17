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


@Autonomous (name="OTCTMAuto", group="FTCPio")
public class OTCTMAuto extends LinearOpMode
{
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;

    // State used for updating telemetry
    private int choose;


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
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

        CVManager tFlow = new CVManager();
        CamManager camM = new CamManager();
        tFlow.init(robot.webCam, robot.tFlowId, camM);
        camM.init(robot, tFlow);
        camM.reference = angles.firstAngle;
        tFlow.autoDisable = true;
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


        while (robot.linearArm.getCurrentPosition()<= 4500)  /* Most effective detachment point might not be at the top*/ {
            //positive = up
            telemetry.addData("Status: ", "Lowering robot");
            telemetry.addData("Encoder: ", robot.linearArm.getCurrentPosition());
            telemetry.addData("Bottom is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.status);
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
            telemetry.addData("Status:","%d",tFlow.status);
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
        telemetry.addData("Status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();

        //Drive away
        robot.encoderDrive(0.5,5,10);

        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();
        runtime.reset();
        while (tFlow.go && runtime.milliseconds() <= 2000) {
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.update();
            sleep(1);
        }

        // Tensorflow output stored into variable to choose where robot should go
        choose = (int) tFlow.minDat[0];
        camM.go = false;
        tFlow.go = false;
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();

        // Perform sampling and position for dropping team marker
        //sleep(6000);
        switch (choose) {
            case 1:
                //Mineral on Left
                robot.angleTurn(TURN_SPEED,33);
                robot.encoderDrive(DRIVE_SPEED,31, 5);
                robot.angleTurn(0.3, -83);
                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                robot.encoderDrive(DRIVE_SPEED,13, 5);

                break;
            case 2:
                //Mineral in Middle
                //no turning movement is necessary to hit mineral
                robot.encoderDrive(DRIVE_SPEED,38, 5);

                break;
            case 3:
                //Mineral on Right
                robot.angleTurn(TURN_SPEED,-38);
                robot.encoderDrive(DRIVE_SPEED,33, 5);
                robot.angleTurn(0.3, 63);
                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                robot.encoderDrive(DRIVE_SPEED,15,5);

                break;
            default:
                //error happened with TensorFlow
                telemetry.addData("TFlow says: ", "%d",tFlow.status);
                // if tensor flow doesn't function, the robot will default to roboting to the middle position
                robot.encoderDrive(DRIVE_SPEED,13,5);
                robot.encoderDrive(DRIVE_SPEED,10, 5);

                robot.angleTurn(TURN_SPEED, 90);
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
        // Back up into crater at max speed

        switch (choose) {
            case 1:
                robot.angleTurn(0.1, -12);
                robot.encoderDrive(1,-53, 10);
                break;
            case 2:
                robot.angleTurn(TURN_SPEED, 75);
                robot.encoderDrive(DRIVE_SPEED,9, 5);
                robot.angleTurn(TURN_SPEED, 35);
                robot.encoderDrive(1,55, 10);
                break;
            case 3:
                robot.encoderDrive(DRIVE_SPEED, -5,5);
                robot.angleTurn( TURN_SPEED, 40);
                robot.encoderDrive(DRIVE_SPEED, 22, 5);
                robot.angleTurn(TURN_SPEED, 35);
                robot.encoderDrive(1,53, 10);
                break;
        }
        if (this.isStopRequested()) {
            robot.linearArm.setPower(0);
            return;
        }
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
                return;
            }

        }
        robot.linearArm.setPower(0);
        telemetry.addData("Status: ", "Finished");
        telemetry.update();
    }
}

