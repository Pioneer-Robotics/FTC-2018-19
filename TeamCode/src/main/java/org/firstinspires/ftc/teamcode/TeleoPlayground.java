package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TeleoPlayground", group="FTCPio")
public class TeleoPlayground extends LinearOpMode
{
    /* Declare OpMode members. */
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime runtime = new ElapsedTime();
    private static final double     TETRIX_TICKS_PER_REV    = 1440;
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    private static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;
    private static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private static final double DRIVE_SPEED = 0.5;

    // State used for updating telemetry
    private Orientation angles;

    @Override
    public void runOpMode()
    {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double arm;
        double armMax;
        double pre_suq = 0;
        boolean flipster = false;
        boolean flipster1 = false;
        int activate_suq = 0;

        CamManager camM = new CamManager();
        //Acceleration gravity = imu.getGravity();
        CVManager tFlow = new CVManager();

        robot.init(hardwareMap);

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.botSwitch.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        camM.init(robot.imu,robot);
        camM.reference = angles.firstAngle;
        camM.start();
        tFlow.start();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;
            arm = gamepad2.right_stick_y;

            telemetry.addData("TFlow says: ", "%d", tFlow.Status);
            telemetry.addData("TFlow saysX: ", "%.5f", tFlow.mineralX);

            //normalization of arm value
            armMax = Math.abs(arm);
            if (armMax > 1.0)
            {
                arm /= armMax;
            }
            if (gamepad1.a) {
                encoderDrive(0.5,35,35,10);
            }
            if (gamepad1.b) {
                encoderDrive(0.5,5,5,10);
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
                angleTurn(0.3,180);
                angleTurn(0.3, -45);
                while (!robot.botSwitch.getState()) {
                    robot.linearArm.setPower(-1);
                }
                robot.linearArm.setPower(0);
                break;
            }
            telemetry.addData("Trigger is", robot.trigger.isPressed() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            // Output the safe vales to the motor drives.
            if ((!robot.botSwitch.getState() && !robot.topSwitch.getState() && !robot.trigger.isPressed()))
            {
                robot.linearArm.setPower(-arm);
            }
            else if ((robot.botSwitch.getState() || robot.trigger.isPressed()) && arm < 0)
            {
                robot.linearArm.setPower(-arm);
            }
            else if ((robot.botSwitch.getState() || robot.trigger.isPressed()) && arm >= 0)
            {
                robot.linearArm.setPower(0);
            }
            else if (arm > 0)
            {
                robot.linearArm.setPower(-arm);
            }
            else if (robot.topSwitch.getState() && arm <= 0)
            {
                robot.linearArm.setPower(0);
            }

            //blended motion
            left  = (drive + turn/2)*(1-gamepad1.right_trigger);
            right = (drive - turn/2)*(1-gamepad1.right_trigger);

            // Normalize the values so neither exceed +/- 1.0
            max = (Math.max(Math.abs(left), Math.abs(right)))/2;
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            if (gamepad1.y)
            {
                left *= -0.25;
                right *= -0.25;
                robot.motorLeft.setPower(right);
                robot.motorRight.setPower(left);
                telemetry.addData("Reverse","Activated");
            }
            else
            {
                robot.motorLeft.setPower(left);
                robot.motorRight.setPower(right);
                telemetry.addData("Reverse", "Deactivated");
            }
            if (gamepad2.right_bumper) {
                if (!flipster) {
                    if (activate_suq == 0) {
                        robot.Collector.setPosition(1);
                        activate_suq = 1;
                        robot.Succq.setPower(activate_suq);
                        sleep(20);
                    } else {
                        activate_suq = 0;
                    }
                    flipster = true;
                }
            } else {
                flipster = false;
            }
            if (gamepad2.left_bumper) {
                if (!flipster1) {
                    activate_suq = -activate_suq;
                }
                flipster1 = true;
            } else {
                flipster1 = false;
            }
            //if the Succq isn't moving then stop it to save the motor
            if ((activate_suq!=0) && (pre_suq == robot.Succq.getCurrentPosition())) {
                activate_suq = 0;
            }
            robot.Succq.setPower(activate_suq);
            pre_suq = robot.Succq.getCurrentPosition();
            telemetry.addData("IMU Heading:", "%.5f", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+180);
            telemetry.addData("Camera:", "%.3f",turn);

            if (gamepad1.dpad_left) {
                angleTurn(0.3,90);
            }
            if (gamepad1.dpad_right) {
                angleTurn(0.3,-90);
            }
            // Controls latching servos on linear actuator
            // Latch open
            if (gamepad2.dpad_up)
            {
                robot.Latch.setPosition(HardwareInfinity.LatchMAX_POSITION);
                telemetry.addData("Latches","Max");
            }
            // Latch closed
            if (gamepad2.dpad_down)
            {
                robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
                telemetry.addData("Latches","Min");
            }
            // Drops team marker with servo
            if (gamepad2.x)
            {
                robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
                sleep(1000);
                robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
                telemetry.addLine("Team Marker Dropped");
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("arm","%.2f", arm);
            telemetry.addData("Motor Encoder", "%d",robot.linearArm.getCurrentPosition());
            telemetry.addData("Camera:","%.5f",robot.Camera.getPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
        if (this.isStopRequested()) {
            tFlow.go = false;
            camM.go = false;
        }
    }
    private void angleTurn(double speed, double angle) {
        double targetAngle;
        int margin = 7;
        if (opModeIsActive()) {
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
            return;
        }
        return;
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
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > initAng+5
                        || robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle < initAng-5) {

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

}
