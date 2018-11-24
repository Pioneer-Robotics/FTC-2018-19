package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="LL5156: Auto Drive by Encoder", group="LL5156")
public class SimpleAuto extends LinearOpMode
{
    HardwareLL5156 robot = new HardwareLL5156();
    private ElapsedTime runtime = new ElapsedTime();
    //private Gyroscope imu;
    //private DcMotor motorLeft;
    //private DcMotor motorLeft;

    static final double     TETRIX_TICKS_PER_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.4;
    /*public double Rotations(double Rotation)
    {
        return Rotation*TETRIX_TICKS_PER_REV;
    }

    public void Tank(double powerL,double powerR)
    {
        motorRight.setPower(powerR);
        motorLeft.setPower(powerL);
    }*/
    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        /*imu = hardwareMap.get(Gyroscope.class, "imu");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");*/

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Tank(1,1);
        //(motorLeft.getCurrentPosition() < Rotations(2))

        telemetry.addData("Path0", "Starting at %7d:%7d",
                robot.motorLeft.getCurrentPosition(),robot.motorRight.getCurrentPosition());
        telemetry.update();


        //ACTUAL MOVEMENT

        //Drop down off lander - lowering robot
        while (robot.topSwitch.getState())  /* Most effective detachment point might not be at the top*/
        {
            //positive = up
            telemetry.addData("Status: ", "Lowering");
            telemetry.update();
            robot.linearArm.setPower(1);

        }
        robot.linearArm.setPower(0);

        // Detach from lander
        robot.Latch.setPosition(HardwareLL5156.LatchMAX_POSITION);
        telemetry.addData("Latches","Max");
        telemetry.addData("Status: ", "Disengaging From Lander");
        telemetry.update();

        //Drive away
        encoderDrive(DRIVE_SPEED,-10,-10,5.0);

        //Run Google Tensor Flow to detect object.....
        telemetry.addData("Status: ", "Detecting Gold Sample");
        telemetry.update();




        encoderDrive(TURN_SPEED,16,-16,5.0);
        //NEED TO TEST MORE, (16,-16) is close to 90 degrees
        //encoderDrive(DRIVE_SPEED,24,24,4.0);
        sleep(250);

        //Drop Team Marker
        telemetry.addData("Status: ","Dropping Team Marker");
        telemetry.update();
        robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMIN_POSITION);
        telemetry.addData("Status: ","Dropped Team Marker");
        telemetry.update();
        sleep(500);
        robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMAX_POSITION);



        while (opModeIsActive())
        {
            telemetry.addData("Power L", robot.motorLeft.getPower());
            telemetry.addData("Power R", robot.motorRight.getPower());
            telemetry.addData("Rotations L",robot.motorLeft.getCurrentPosition()/TETRIX_TICKS_PER_REV);
            telemetry.addData("Rotations R",robot.motorLeft.getCurrentPosition()/TETRIX_TICKS_PER_REV);
            telemetry.addData("Linear Encoder", robot.linearArm.getCurrentPosition());
            telemetry.update();
        }
        //Tank(0,0);
        telemetry.addData("Status: ","Finished");
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS )
    {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive())
        {
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(leftCM*COUNTS_PER_INCH);
            newRightTarget = robot.motorLeft.getCurrentPosition() + (int)(rightCM*COUNTS_PER_INCH);
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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
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
