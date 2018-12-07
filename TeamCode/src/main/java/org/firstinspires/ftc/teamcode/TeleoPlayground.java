package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    private static final double     TETRIX_TICKS_PER_REV    = 1440;
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    private static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;
    private static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;

    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();


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
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IParameters);
        /*imu = hardwareMap.get(Gyroscope.class, "imu");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");*/
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Acceleration gravity = imu.getGravity();
        CVManager tFlow = new CVManager();

        robot.init(hardwareMap);

        //tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.botSwitch.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        camM.init(imu,robot);
        camM.reference = angles.firstAngle;
        //camM.start();
        //tFlow.start();


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
                robot.motorLeft.setPower(-right);
                robot.motorRight.setPower(-left);
                telemetry.addData("Reverse","Activated");
            }
            else
            {
                robot.motorLeft.setPower(-left);
                robot.motorRight.setPower(-right);
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
            telemetry.addData("IMU Heading:", "%.5f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+180);
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
    }
    private void angleTurn(double speed, double angle) {
        double targetAngle;
        int margin = 7;
        if (opModeIsActive()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetAngle = angle+angles.firstAngle+180;
            telemetry.clearAll();
            while (Math.abs(angles.firstAngle+180-targetAngle) > margin*speed && (360-Math.abs(angles.firstAngle+180-targetAngle)) > margin*speed) {
                if (angle>0) {
                    robot.motorLeft.setPower(Math.abs(speed));
                    robot.motorRight.setPower(-Math.abs(speed));
                } else {
                    robot.motorLeft.setPower(-Math.abs(speed));
                    robot.motorRight.setPower(Math.abs(speed));
                }
                telemetry.addData("Error:","%.5f", Math.abs(angles.firstAngle+180-targetAngle));
                telemetry.addData("Margin:","%.5f",margin*speed);
                telemetry.addData("IMU Heading:", "%.5f", angles.firstAngle + 180);
                telemetry.addData("min:", "%.5f", targetAngle - margin * speed);
                telemetry.addData("target:", "%.5f", targetAngle);
                telemetry.addData("max:", "%.5f", targetAngle + margin * speed);
                telemetry.update();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);
        }
    }
}
