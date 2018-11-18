package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name="armConfig", group="LL5156")

public class Config {
    HardwareLL5156 robot = new HardwareLL5156();

    public static double           LINEAR_ZERO;
    public static final double     CHANGE_IN_LINEAR        = 5000; // change value to diff
    public static double           MAX_RGT_LATCH           = 0;
    public static double           MIN_RGT_LATCH           = 0;
    public static double           MAX_LFT_LATCH           = 0;
    public static double           MIN_LFT_LATCH           = 0;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.linearSwitch.setMode(DigitalChannel.Mode.INPUT);



        //linear arm to bottom to record "0" value
        if (robot.linearSwitch.getState()) {
            robot.linearArm.setPower(0.75);
            telemetry.addData("Switch ", "is calibrating");
            telemetry.update();

        } else if (!robot.linearSwitch.getState()) {
            robot.linearArm.setPower(0);
            LINEAR_ZERO = robot.linearArm.getCurrentPosition();
            telemetry.addData("Switch ", "is calibrated");
            telemetry.update();

        }


        //calibrating right latch
        if (gamepad2.start)
        {


            while (!gamepad2.atRest())
            { /*ensures separation between steps*/}

            while (!gamepad2.start){

                /*----------------------------------------------rgtLatch_Config Start--------------------------------------------------------------------*/
                if ((-gampad2.right_stick_y) > 0){
                    robot.rgtLatch.setPosition(robot.rgtLatch.getCurrentPosition() + ((-gampad2.right_stick_y) * 0.1) );
                    telemetry.addData("Right Latch: ", "is calibrating");
                    telemetry.update();
                }


                if ((-gampad2.right_stick_y) < 0){
                    robot.rgtLatch.setPosition(robot.rgtLatch.getCurrentPosition() - ((-gampad2.right_stick_y) * 0.1) );
                    telemetry.addData("Right Latch: ", "is calibrating");
                    telemetry.update();
                }


                if (gamepad2.right_trigger && gamepad2.dpad_down){
                    robot.rgtLatch.getCurrentPosition() = MIN_RGT_LATCH;
                }
                if (gamepad2.right_trigger && gamepad2.dpad_up){
                    robot.rgtLatch.getCurrentPosition() = MAX_RGT_LATCH;
                }

                /*----------------------------------------------END--------------------------------------------------------------------------------*/
                /*----------------------------------------------lftLatch_Config Start--------------------------------------------------------------------*/
                if ((-gampad2.right_stick_y) > 0){
                    robot.lftLatch.setPosition(robot.lftLatch.getCurrentPosition() + ((-gampad2.left_stick_y) * 0.1) );
                    telemetry.addData("Left Latch: ", "is calibrating");
                    telemetry.update();
                }


                if ((-gampad2.left_stick_y) < 0){
                    robot.lftLatch.setPosition(robot.lftLatch.getCurrentPosition() - ((-gampad2.left_stick_y) * 0.1) );
                    telemetry.addData("Left Latch: ", "is calibrating");
                    telemetry.update();
                }


                if (gamepad2.left_trigger && gamepad2.dpad_down){
                    robot.lftLatch.getCurrentPosition() = MIN_LFT_LATCH;
                }
                if (gamepad2.left_trigger && gamepad2.dpad_up){
                    robot.lftLatch.getCurrentPosition() = MAX_LFT_LATCH;
                }
                /*----------------------------------------------END--------------------------------------------------------------------------------*/
                /*----------------------------------------------Telemetry Start--------------------------------------------------------------------*/
                if (MIN_LFT_LATCH != 0){
                    telemetry.addData("Left Latch Min: ", MIN_LFT_LATCH);
                    telemetry.update();
                }
                if (MAX_LFT_LATCH != 0){
                    telemetry.addData("Left Latch Max: ", MAX_LFT_LATCH);
                    telemetry.update();
                }
                if (MIN_RGT_LATCH != 0){
                    telemetry.addData("Right Latch Min: ", MIN_RGT_LATCH);
                    telemetry.update();
                }
                if (MAX_RGT_LATCH != 0){
                    telemetry.addData("Right Latch Max: ", MAX_RGT_LATCH);
                    telemetry.update();
                }
                /*----------------------------------------------END--------------------------------------------------------------------------------*/
            }


        }
        //set left to max/min
        if (gamepad2.left_bumper && gamepad2.dpad_down){
            robot.lftLatch.setPosition(MIN_LFT_LATCH);
        }
        if (gamepad2.left_bumper && gamepad2.dpad_up){
            robot.lftLatch.setPosition(MAX_LFT_LATCH);
        }


        //set right to max/min
        if (gamepad2.right_bumper && gamepad2.dpad_down){
            robot.rgtLatch.setPosition(MIN_RGT_LATCH);
        }
        if (gamepad2.right_bumper && gamepad2.dpad_up){
            robot.rgtLatch.setPosition(MAX_RGT_LATCH);
        }






    }


}
