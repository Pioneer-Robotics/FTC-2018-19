package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Autonomous (name="TMAutoNL", group="FTCPio")
@Disabled
public class TMAutoNL extends LinearOpMode {
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
        Movement mov = new Movement();
        tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        camM.init(robot.imu,robot, hardwareMap, tFlow);
        mov.init(robot.motorLeft,robot.motorRight,robot.imu,this,runtime, COUNTS_PER_INCH);
        camM.reference = angles.firstAngle;
        camM.start();
        tFlow.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Path0", "Starting at %7d:%7d",
                robot.motorLeft.getCurrentPosition(), robot.motorRight.getCurrentPosition());
        telemetry.update();
        tFlow.disable = true;


        //Drop down off lander - lowering robot


        /*     ACTUAL MOVEMENT--------------------------------------------------------------*/

        // Detach from lander
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        telemetry.addData("Latches", "Min");
        telemetry.addData("Status: ", "Disengaging From Lander");
        telemetry.update();
        while (!robot.botSwitch.getState() && !robot.topSwitch.getState()) {
            robot.linearArm.setPower(1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
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
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.Status);
        telemetry.addData("MineralX:","%.5f",tFlow.mineralX);
        telemetry.update();

        //Drive away
        mov.encoderDrive(0.5,5,5,30);
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.Status);
        telemetry.addData("MineralX:","%.5f",tFlow.mineralX);
        telemetry.update();
        runtime.reset();
        while (tFlow.go && runtime.milliseconds() <= 2000) {
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:","%d",tFlow.Status);
            telemetry.addData("MineralX:","%.5f",tFlow.mineralX);
            telemetry.update();
            sleep(1);
        }
        choose = tFlow.Status;
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
        camM.go = false;
        tFlow.go = false;
        telemetry.addData("Choose:", "%d", choose);
        telemetry.addData("Status:","%d",tFlow.Status);
        telemetry.addData("MineralX:","%.5f",tFlow.mineralX);
        telemetry.update();
        switch (choose) {
            case 1:
                //Mineral on Left
                mov.angleTurn(0.5,23);

                mov.encoderDrive(DRIVE_SPEED, 35, 35, 5);
                mov.angleTurn(0.2, -76);
                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                mov.encoderDrive(DRIVE_SPEED,25,25,30);
                mov.angleTurn(0.3, 80);

                break;
            case 2:
                //Mineral in Middle
                //no turning movement is necessary to hit mineral

                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                mov.encoderDrive(DRIVE_SPEED, 45, 45, 5);

                mov.angleTurn(0.2, 60);

                telemetry.addData("Status: ", "Dropping Team Marker");
                telemetry.update();

                //mov.encoderDrive( 0.5, 5,5,10);
                break;
            case 3:
                //Mineral on Right
                mov.angleTurn(0.5,-23);

                mov.encoderDrive(DRIVE_SPEED, 35, 35, 5);
                mov.angleTurn(0.2, 66);
                //telemetry.addData("TFlow says: ", "%d",tFlow.Status);
                mov.encoderDrive(DRIVE_SPEED,20,20,30);
                mov.angleTurn(0.2, 40);


                //mov.angleTurn(0.2, 90);
                //mov.encoderDrive( 0.5, 5,5,10);
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
                mov.encoderDrive(DRIVE_SPEED, 13, 13, 5);
                mov.encoderDrive( 0.5,10,10,10);

                mov.angleTurn(0.3, 90);
                break;
        }
        telemetry.addData("Status: ", "Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
        sleep(1500);
        telemetry.addData("Status: ", "Dropped Team Marker");
        telemetry.update();
        sleep(1500);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        telemetry.update();

        /*
        mov.angleTurn(0.3,160);
        mov.angleTurn(0.3, -25);
        */
        robot.Camera.setPosition(0);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        while (!robot.botSwitch.getState() && !robot.topSwitch.getState()) {
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
