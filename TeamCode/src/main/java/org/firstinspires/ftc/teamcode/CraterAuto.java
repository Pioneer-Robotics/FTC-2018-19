package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous (name="CraterAuto", group="FTCPio")
public class CraterAuto extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SPEED = 0.5;
    private static final boolean latch = true;

    CVManager tFlow = new CVManager();
    CamManager camM = new CamManager();

    // State used for updating telemetry
    private int choose;

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

        tFlow.init(robot.webCam, robot.tFlowId, camM);
        camM.init(robot, tFlow);
        camM.reference = angles.firstAngle;
        tFlow.autoDisable = true;
        camM.start();
        camM.mode = 0;

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

        if (latch) {
            robot.armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.linearArm.getCurrentPosition() <= 4700)  /* Most effective detachment point might not be at the top*/ {
                //positive = up
                telemetry.addData("status: ", "Lowering robot");
                telemetry.addData("Encoder: ", robot.linearArm.getCurrentPosition());
                telemetry.addData("Bottom is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
                telemetry.addData("Choose:", "%d", choose);
                telemetry.addData("status:", "%d", tFlow.status);
                telemetry.addData("Tar:","%d",tFlow.tar);
                telemetry.update();
                robot.linearArm.setPower(1);
                if (robot.topSwitch.getState()) {
                    robot.linearArm.setPower(0);
                    break;
                }
            /*if (robot.armBase.getCurrentPosition() < 500) robot.armBase.setPower(-1);
            else robot.armBase.setPower(0);*/
                if (this.isStopRequested()) {
                    robot.linearArm.setPower(0);
                    camM.go = false;
                    tFlow.go = false;
                    return;
                }

            }
            robot.linearArm.setPower(0);
        }
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
                telemetry.addData("status:", "%d", tFlow.status);
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
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("status:","%d",tFlow.status);
        telemetry.addData("Tar:","%d",tFlow.tar);
        telemetry.update();
        sleep(500);
        //Drive away
        robot.encoderDrive(DRIVE_SPEED,5,10);

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
        sleep(100);

        // Perform sampling
        switch (choose) {
            case 1:
                //left
                robot.angleTurn(TURN_SPEED,38, true);

                robot.encoderDrive(DRIVE_SPEED, 17, 5);
                sleep(100);
                /*
                robot.angleTurn(TURN_SPEED,40);
                robot.angleTurn(TURN_SPEED,-40);
                */
                robot.encoderDrive(DRIVE_SPEED, -17,5);

                robot.angleTurn(0.3,-38);

                break;
            case 2:
                //middle
                //theoretically no lateral movement is necessary
                robot.encoderDrive(DRIVE_SPEED,12, 5);
                sleep(100);
                robot.encoderDrive(DRIVE_SPEED,-12, 5);


                break;
            case 3:
                //right
                robot.angleTurn(TURN_SPEED,-38, true);

                robot.encoderDrive(DRIVE_SPEED,17, 5);
                sleep(100);
                /*
                robot.angleTurn(TURN_SPEED,-40);
                robot.angleTurn(TURN_SPEED,40);
                */
                robot.encoderDrive(DRIVE_SPEED, -17,5);
                robot.angleTurn(0.2,38);

                break;
            default:
                //error happened with TensorFlow
                telemetry.addData("TFlow says: ", "%d",tFlow.status);
                // if tensor flow doesn't function, the robot will default to roboting to the middle position
                robot.encoderDrive(DRIVE_SPEED,12, 5);
                sleep(100);
                robot.encoderDrive(DRIVE_SPEED,-12, 5);
                break;
        }

        // Maneuver to depot to drop team marker
        //sleep(250);
        robot.encoderDrive(0.2, 3,5);

        robot.angleTurn(0.2,68);
        robot.encoderDrive(DRIVE_SPEED, 34,5);
        robot.angleTurn(0.2,41);
        robot.encoderDrive(DRIVE_SPEED, 34,5);

        //sleep(500);
        telemetry.update();
        telemetry.addData("status: ", "Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
        telemetry.addData("status: ", "Dropped Team Marker");
        telemetry.update();
        sleep(500);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);

        // Align with wall and back up into crater
        //robot.angleTurn(0.1, 10);
        robot.encoderDrive(DRIVE_SPEED, -2,5);
        robot.encoderDrive(DRIVE_SPEED, -60, 5);

        //robot.angleTurn(TURN_SPEED,95, false);
        //robot.encoderDrive(DRIVE_SPEED, 70, 20);
        if (this.isStopRequested())
        {
            robot.linearArm.setPower(0);
            return;
        }
        robot.Camera.setPosition(0);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        if (false) {
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
        }


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
        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles1   = robot.imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("status: ", "Finished");
        telemetry.addData("Drift:", "%.10f", angles.firstAngle-angles1.firstAngle);
        telemetry.update();
        if (this.isStopRequested()) {
            robot.linearArm.setPower(0);
            return;
        }
    }
}
