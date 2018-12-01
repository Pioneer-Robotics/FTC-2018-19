package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="LL5156:POV", group="LL5156")
public class Teleop extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareLL5156 robot           = new HardwareLL5156();
    static final double     TETRIX_TICKS_PER_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;
    static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();


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
        double cam;
        boolean flipster = false;
        int activate_suq = 0;

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
        gravity = imu.getGravity();

        robot.init(hardwareMap);

        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.botSwitch.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;
            arm = gamepad2.right_stick_y;
            cam = gamepad2.left_stick_x;

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
            left  = drive + turn/2;
            right = drive - turn/2;

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
            if (gamepad2.a) {
                if (!flipster) {
                    if (activate_suq == 0) {
                        robot.Collector.setPosition(0);
                        activate_suq = 1;
                    } else {
                        activate_suq = -activate_suq;
                    }
                    flipster = true;
                }
            } else {
                flipster = false;
            }
            if (gamepad2.left_bumper) {
                activate_suq = 0;
            }
            robot.Succq.setPower(activate_suq);
            telemetry.addData("IMU Heading:", "%.5f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            robot.Camera.setPosition((-turn+0.9)/2);
            telemetry.addData("Camera:", "%.3f",turn);

            if (gamepad1.dpad_left) {
                angleTurn(0.5,-90);
            }
            if (gamepad1.dpad_right) {
                angleTurn(0.5,90);
            }
            // Controls latching servos on linear actuator
            // Latch open
            if (gamepad2.dpad_up)
            {
                robot.Latch.setPosition(HardwareLL5156.LatchMAX_POSITION);
                telemetry.addData("Latches","Max");
            }
            // Latch closed
            if (gamepad2.dpad_down)
            {
                robot.Latch.setPosition(HardwareLL5156.LatchMIN_POSITION);
                telemetry.addData("Latches","Min");
            }
            // Drops team marker with servo
            if (gamepad2.x)
            {
                robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMIN_POSITION);
                sleep(1000);
                robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMAX_POSITION);
                telemetry.addLine("Team Marker Dropped");
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("arm","%.2f", arm);
            telemetry.addData("Motor Encoder", "%d",robot.linearArm.getCurrentPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }
    public void angleTurn(double speed, double angle) {
        double targetAngle;
        if (opModeIsActive()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            targetAngle = angle+angles.firstAngle;
            while (angles.firstAngle+180 > (targetAngle+180+20*speed)%360 || angles.firstAngle < (targetAngle+180-20*speed)%360) {
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
}
