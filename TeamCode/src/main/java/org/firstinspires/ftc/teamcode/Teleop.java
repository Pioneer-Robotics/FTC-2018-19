package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="FTCPio")
public class Teleop extends OpMode
{
    /* Declare OpMode members. */
    private HardwareInfinity robot = new HardwareInfinity();
    ElapsedTime time = new ElapsedTime();
    double left;
    double right;
    double drive;
    double armB;
    double bar;
    double turn;
    double max;
    double arm;
    double pre_suq = 0;
    boolean armBAuto;
    boolean FBarAuto;
    boolean dmac = false;
    boolean flipster = true;
    double activate_suq = 0;
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
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        robot.armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
    }
    @Override
    public void start() {
        time.reset();
    }
    @Override
    public void loop()
    {
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = gamepad1.left_stick_y;
        turn  =  -gamepad1.left_stick_x;
        armB = -gamepad2.left_stick_y/4*(1+3*gamepad2.left_trigger);
        bar = gamepad2.right_stick_y;
        if (asuq==0) activate_suq = -Math.copySign(Math.pow(gamepad1.right_stick_y,2),gamepad1.right_stick_y);
        telemetry.addData("Succq:", activate_suq);
        telemetry.addData("Succq Encoder: ", "%d", robot.Succq.getCurrentPosition());
        telemetry.addData("Condition: ", "%.5f", Math.abs(robot.dropTop.getPosition() - HardwareInfinity.DT_MIN));
        telemetry.addData("Flipster: ", flipster ? "True" : "False");
        telemetry.addData("DT pos: ", robot.dropTop.getPosition());
        if (gamepad1.left_bumper) {
            arm = 1;
        } else if (gamepad1.right_bumper) {
            arm = -1;
        } else arm = 0;
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
        left  = (drive+turn)/2*(gamepad1.right_trigger*3/2+1)*0.75;
        right = (drive-turn)/2*(gamepad1.right_trigger*3/2+1)*0.75;
        telemetry.addData("Multiplier:", (gamepad1.right_trigger*3/2+1)*0.75);

        // Normalize the values so neither exceed +/- 1.0
        max = (Math.max(Math.abs(left), Math.abs(right)))/2;
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Decrease power input and flip direction so line-up for reattachment on lander is easier
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

        /*if (gamepad1.right_bumper) {
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
        }*/
        /*if (gamepad2.left_bumper) robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        else robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.right_bumper) robot.FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        else robot.FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        /*if (gamepad1.left_bumper) {
            if (!flipster1) {
                asuq = -asuq;
            }
            flipster1 = true;
        } else {
            flipster1 = false;
        }*/

        if (gamepad1.a) {
            if (flipster) {
                if (Math.abs(robot.dropTop.getPosition() - HardwareInfinity.DT_MIN)<=0.1) {
                    robot.dropTop.setPosition(HardwareInfinity.DT_MAX);
                } else robot.dropTop.setPosition(HardwareInfinity.DT_MIN);
            }
            flipster = false;
        } else {
            flipster = true;
        }
        if (robot.armBase.getCurrentPosition()<-3000 && !gamepad1.b) {
            robot.dropTop.setPosition(HardwareInfinity.DT_MAX);
        }
        //if the Succq isn't moving then stop it to save the motor
        if (asuq != 0) activate_suq = asuq;
        if ((activate_suq!=0) && (pre_suq == robot.Succq.getCurrentPosition()) && gamepad1.right_stick_y==0) {
            activate_suq = 0;
        }
        robot.Succq.setPower(activate_suq);
        pre_suq = robot.Succq.getCurrentPosition();
        telemetry.addData("Arm Base Power: ","%.5f",armB);
        telemetry.addData("Arm Base Encoder: ", "%d", robot.armBase.getCurrentPosition());
        if (!armBAuto) {
            robot.armBase.setPower(armB);
        } else if (armB!=0 || gamepad2.left_bumper || gamepad2.a) {
            robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armBase.setPower(0);
            armBAuto = false;
            dmac = false;
        }
        telemetry.addData("4Bar Power: ","%.5f",bar);
        telemetry.addData("4Bar Encoder: ", "%d", robot.FBar.getCurrentPosition());
        if (!FBarAuto) {
            robot.FBar.setPower(bar);
        } else if (bar!=0 || gamepad2.right_bumper|| gamepad2.a) {
            robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FBar.setPower(0);
            FBarAuto = false;
            dmac = false;
        }
        // Controls latching servos on linear actuator
        // Latch open
        if (gamepad1.dpad_right)
        {
            robot.Latch.setPosition(HardwareInfinity.LatchMAX_POSITION);
            telemetry.addData("Latches","Max");
        }
        // Latch closed
        if (gamepad1.dpad_left)
        {
            robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
            telemetry.addData("Latches","Min");
        }
        if ((gamepad1.dpad_down || gamepad2.dpad_down) && robot.dropTop.getPosition()>HardwareInfinity.DT_MIN) {
            robot.dropTop.setPosition(robot.dropTop.getPosition()-0.015);
        }
        if ((gamepad1.dpad_up || gamepad2.dpad_up) && robot.dropTop.getPosition()<HardwareInfinity.DT_MAX) {
            robot.dropTop.setPosition(robot.dropTop.getPosition()+0.015);
        }

        // The following gamepad2 inputs are macros to move the EMCD 2.0 between its collection and depositing positions
        if (gamepad2.x) {
            armBAuto = true;
            robot.armBase.setTargetPosition(-300);
            robot.armBase.setPower(-1);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FBarAuto = true;
            robot.FBar.setTargetPosition(-25000);
            robot.FBar.setPower(-1);
            robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dmac = true;
            time.reset();
        }

        if (gamepad2.a) {
            armBAuto = true;
            robot.armBase.setTargetPosition(-300);
            robot.armBase.setPower(1);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FBarAuto = true;
            robot.FBar.setTargetPosition(-300);
            robot.FBar.setPower(1);
            robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.y) {
            armBAuto = true;
            robot.armBase.setTargetPosition(-300);
            robot.armBase.setPower(-1);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FBarAuto = true;
            robot.FBar.setTargetPosition(-20500);
            robot.FBar.setPower(-1);
            robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Time: ", "%.5f", time.milliseconds());
        telemetry.addData("dmac:", "%b", dmac);
        if (dmac && (time.milliseconds() >= 4000 || (!robot.armBase.isBusy() && !robot.FBar.isBusy()))) {
                robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armBase.setTargetPosition(-6750);
                robot.armBase.setPower(-1);
                robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.FBar.setTargetPosition(-24750);
                robot.FBar.setPower(-1);
                robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dmac = false;
        }
        if (robot.armBase.isBusy()) {
            armBAuto = true;
        } else if (armBAuto) {
            armBAuto = false;
            robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (robot.FBar.isBusy()) {
            FBarAuto = true;
        } else if (FBarAuto) {
            FBarAuto = false;
            robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
         // Drops team marker with servo
        if (gamepad1.x)
        {
            robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
            try {
                Thread.sleep(1000);
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
