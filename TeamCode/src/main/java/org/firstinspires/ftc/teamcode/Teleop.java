package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="TeleOp", group="FTCPio")
public class Teleop extends LinearOpMode
{
    /* Declare OpMode members. */
    private HardwareInfinity robot = new HardwareInfinity();
    private static final double     TETRIX_TICKS_PER_REV    = 1440;
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    private static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;
    private static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    // State used for updating telemetry;

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

        /*imu = hardwareMap.get(Gyroscope.class, "imu");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");*/
        //Acceleration gravity = imu.getGravity();

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
                left *= -0.5*(1-gamepad1.right_trigger);
                right *= -0.5*(1-gamepad1.right_trigger);
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
            if (gamepad2.b) {
                robot.Collector.setPosition(1);
            }
            if (gamepad2.right_bumper) {
                if (!flipster) {
                    if (activate_suq == 0) {
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

            /*
             // Drops team marker with servo
            if (gamepad2.x)
            {
                robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
                sleep(50);
                robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
                telemetry.addLine("Team Marker Dropped");
            }
            */

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("arm","%.2f", arm);
            telemetry.addData("Motor Encoder", "%d",robot.linearArm.getCurrentPosition());
            telemetry.update();
        }
    }
}
