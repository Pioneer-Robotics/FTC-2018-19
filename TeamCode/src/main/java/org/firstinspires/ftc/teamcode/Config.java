package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name="armConfig", group="FTCPio")
@Disabled
public class Config extends LinearOpMode
{
    private HardwareInfinity robot = new HardwareInfinity();

    private static double           LINEAR_ZERO;
    public static final double     CHANGE_IN_LINEAR        = 5000; // change value to diff
    private static double           MAX_RGT_LATCH           = 0;
    private static double           MIN_RGT_LATCH           = 0;
    private static double           MAX_LFT_LATCH           = 0;
    private static double           MIN_LFT_LATCH           = 0;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.botSwitch.setMode(DigitalChannel.Mode.INPUT);
        robot.topSwitch.setMode(DigitalChannel.Mode.INPUT);



        //linear arm to bottom to record "0" value
        if (robot.botSwitch.getState()) {
            robot.linearArm.setPower(0.75);
            telemetry.addData("Switch ", "is calibrating");
            telemetry.update();

        } else if (!robot.botSwitch.getState()) {
            robot.linearArm.setPower(0);
            LINEAR_ZERO = robot.linearArm.getCurrentPosition();
            telemetry.addData("Switch ", "is calibrated");
            telemetry.update();

        }


        //calibrating right latch
        if (gamepad2.start)
        {


            while (!gamepad2.atRest())

            while (!gamepad2.start){

                /*----------------------------------------------rgtLatch_Config Start--------------------------------------------------------------------*/
                if ((-gamepad2.right_stick_y) > 0){
                    robot.Latch.setPosition(robot.Latch.getPosition() + ((-gamepad2.right_stick_y) * 0.1) );
                    telemetry.addData("Right Latch: ", "is calibrating");
                    telemetry.update();
                }


                if ((-gamepad2.right_stick_y) < 0){
                    robot.Latch.setPosition(robot.Latch.getPosition() - ((-gamepad2.right_stick_y) * 0.1) );
                    telemetry.addData("Right Latch: ", "is calibrating");
                    telemetry.update();
                }

                //BIG PROBLEMS HEREEEEEEEEEE
                if (gamepad2.right_trigger == 1 && gamepad2.dpad_down){
                     MIN_RGT_LATCH = robot.Latch.getPosition();
                }
                if (gamepad2.right_trigger == 1 && gamepad2.dpad_up){
                    MAX_RGT_LATCH = robot.Latch.getPosition();
                }

                /*----------------------------------------------END--------------------------------------------------------------------------------*/
                /*----------------------------------------------lftLatch_Config Start--------------------------------------------------------------------*/
                /*if ((-gampad2.right_stick_y) > 0){
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
                }*/
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
        /*if (gamepad2.left_bumper && gamepad2.dpad_down){
            robot.lftLatch.setPosition(MIN_LFT_LATCH);
        }
        if (gamepad2.left_bumper && gamepad2.dpad_up){
            robot.lftLatch.setPosition(MAX_LFT_LATCH);
        }*/


        //set right to max/min
        if (gamepad2.right_bumper && gamepad2.dpad_down){
            robot.Latch.setPosition(MIN_RGT_LATCH);
        }
        if (gamepad2.right_bumper && gamepad2.dpad_up){
            robot.Latch.setPosition(MAX_RGT_LATCH);
        }






    }


}
