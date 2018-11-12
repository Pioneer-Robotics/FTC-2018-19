package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

        robot.init(hardwareMap);

        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.linearSwitch.setMode(DigitalChannel.Mode.INPUT);
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

            //normalization of arm value
            armMax = Math.abs(arm);
            if (armMax > 1.0)
            {
                arm /= armMax;
            }

            // Output the safe vales to the motor drives.
            if (robot.linearSwitch.getState())
            {
                robot.linearArm.setPower(-arm);
                telemetry.addData("Switch","is not pressed");
            }
            else if (!robot.linearSwitch.getState() && arm <= 0)
            {
                robot.linearArm.setPower(-arm);
                telemetry.addData("Switch","is pressed");
            }
            else if (!robot.linearSwitch.getState() && arm > 0)
            {
                robot.linearArm.setPower(0);
                telemetry.addData("Switch","is pressed");
            }

            //blended motion
            left  = drive + turn;
            right = drive - turn;

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




            // Controls latching servos on linear actuator
            if (gamepad2.dpad_up)
            {
                robot.rgtLatch.setPosition(HardwareLL5156.rgtLatchMAX_POSITION);
                robot.lftLatch.setPosition(HardwareLL5156.lftLatchMAX_POSITION);
                telemetry.addData("Latches","Max");
            }
            if (gamepad2.dpad_down)
            {
                robot.rgtLatch.setPosition(HardwareLL5156.rgtLatchMIN_POSITION);
                robot.lftLatch.setPosition(HardwareLL5156.lftLatchMIN_POSITION);
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
}
