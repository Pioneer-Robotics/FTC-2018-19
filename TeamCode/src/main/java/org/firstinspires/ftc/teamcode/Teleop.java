package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="TeleOp", group="FTCPio")
public class Teleop extends OpMode
{
    /* Declare OpMode members. */
    private HardwareInfinity robot = new HardwareInfinity();
    private static final double     TETRIX_TICKS_PER_REV    = 1440;
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    private static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;
    private static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    double left;
    double right;
    double drive;
    double armB;
    double bar;
    double turn;
    double max;
    double arm;
    double pre_suq = 0;
    double pre_arm = 0;
    double pre_bar = 0;
    boolean flipster = false;
    boolean flipster1 = false;
    boolean flipster2 = false;
    boolean flipster3 = false;
    boolean deathFlip = false;
    float activate_suq = 0;
    int asuq = 0;

    // State used for updating telemetry;
    @Override
    public void init() {

        /*imu = hardwareMap.get(Gyroscope.class, "imu");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");*/
        //Acceleration gravity = imu.getGravity();

        robot.init(hardwareMap);

        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.Camera.setPosition(0);
        robot.lunchBox.setPosition(0.3);
        // Wait for the game to start (driver presses PLAY)
    }
    @Override
    public void start() {

    }
    @Override
    public void loop()
    {
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = gamepad1.left_stick_y;
        turn  =  -gamepad1.left_stick_x;
        armB = -gamepad2.left_stick_y;
        bar = -gamepad2.right_stick_y/8;
        if (asuq==0) activate_suq = gamepad1.right_stick_y;
        /*if (gamepad2.left_bumper) {
            arm = 1;
        } else if (gamepad2.right_bumper) {
            arm = -1;
        } else arm = 0;*/
        arm = gamepad2.left_trigger;
        if (gamepad2.left_trigger == 0) arm = -gamepad2.right_trigger;
        //telemetry.addData("Trigger is", robot.trigger.isPressed() ? "Pressed" : "not Pressed");
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
        left  = (drive + turn/2)*(gamepad1.right_trigger/3+1)*0.75;
        right = (drive - turn/2)*(gamepad1.right_trigger/3+1)*0.75;

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

        if (gamepad1.right_bumper) {
            if (!flipster) {
                if (asuq == 0) {
                    asuq = 1;
                    activate_suq = asuq;
                    robot.Succq.setPower(activate_suq);
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e){
                    }
                } else {
                    asuq = 0;
                }
                flipster = true;
            }
        } else {
            flipster = false;
        }
        if (!gamepad2.left_bumper) robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        else robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!gamepad2.right_bumper) robot.FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        else robot.FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.a) {
            if (!flipster2) {
                if (!deathFlip) {
                    deathFlip = true;
                } else {
                    deathFlip = false;
                }
                flipster2 = true;
            }
        } else {
            flipster2 = false;
        }

        if (gamepad1.left_bumper) {
            if (!flipster1) {
                asuq = -asuq;
            }
            flipster1 = true;
        } else {
            flipster1 = false;
        }

        if (gamepad1.a) {
            if (!flipster3) {
                if (robot.dropTop.getPosition() == 0) {
                    robot.dropTop.setPosition(0.5);
                } else {
                    robot.dropTop.setPosition(0);
                }
                flipster3 = true;
            }
        } else {
            flipster3 = false;
        }
        //if the Succq isn't moving then stop it to save the motor
        if (asuq != 0) activate_suq = asuq;
        telemetry.addData("Death Flip: ", deathFlip);
        if ((activate_suq!=0) && (pre_suq == robot.Succq.getCurrentPosition()) && gamepad1.right_stick_y==0 && !deathFlip) {
            activate_suq = 0;
        }
        robot.Succq.setPower(activate_suq);
        pre_suq = robot.Succq.getCurrentPosition();
        if ((armB!=0) && (pre_arm == robot.armBase.getCurrentPosition()) && !deathFlip) {
            armB = 0;
        }
        telemetry.addData("Arm Base Power: ","%.5f",armB);
        robot.armBase.setPower(armB);
        pre_arm = robot.armBase.getCurrentPosition();
        if ((bar!=0) && (pre_bar == robot.FBar.getCurrentPosition()) && !deathFlip) {
            bar = 0;
        }
        telemetry.addData("4Bar Power: ","%.5f",bar);
        robot.FBar.setPower(bar);
        pre_bar = robot.Succq.getCurrentPosition();
        // Controls latching servos on linear actuator
        // Latch open
        if (gamepad1.dpad_up)
        {
            robot.Latch.setPosition(HardwareInfinity.LatchMAX_POSITION);
            telemetry.addData("Latches","Max");
        }
        // Latch closed
        if (gamepad1.dpad_down)
        {
            robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
            telemetry.addData("Latches","Min");
        }


         // Drops team marker with servo
        if (gamepad2.x)
        {
            robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
            }
            robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
            telemetry.addLine("Team Marker Dropped");
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("arm","%.2f", arm);
        telemetry.addData("Motor Encoder", "%d",robot.linearArm.getCurrentPosition());
        telemetry.update();
    }
}
