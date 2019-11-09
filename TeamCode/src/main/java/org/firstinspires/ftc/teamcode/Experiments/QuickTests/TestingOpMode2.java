package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "TestingOpMode2", group = "Sensor")
public class TestingOpMode2 extends LinearOpMode {

    Robot robot = new Robot();
    DeltaTime deltaTime = new DeltaTime();


    int last_bl;

    int last_br;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();


        while (opModeIsActive()) {
            deltaTime.Start();


            robot.SetPowerDouble4(1, 1, 1, 1, gamepad1.left_stick_y);


            last_bl = robot.backLeft.getCurrentPosition();
            last_br = robot.backRight.getCurrentPosition();

            telemetry.addData("Encoder Position, BL", robot.backLeft.getPower());
            telemetry.addData("Encoder Position, BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("Encoder Delta, BL", (last_bl - robot.backLeft.getCurrentPosition()) / deltaTime.deltaTime());

            telemetry.addData("Encoder Position, BR", robot.backRight.getPower());
            telemetry.addData("Encoder Position, BR", robot.backRight.getCurrentPosition());
            telemetry.addData("Encoder Delta, BR", (last_br - robot.backRight.getCurrentPosition()) / deltaTime.deltaTime());
            telemetry.update();
            deltaTime.Stop();

//            robot.SetPowerDouble4(1, 0, 0, 0, 0.5);
//            sleep(10000);
//            robot.SetPowerDouble4(0, 0, 0, 0, 0.5);
//            sleep(10000);
//            robot.SetPowerDouble4(-1, 0, 0, 0, 0.5);
//            sleep(10000);
//            robot.SetPowerDouble4(0, 0, 0, 0, 0.5);
//            sleep(10000);

        }
//        robot.SetPowerDouble4(0, 1, 0, 0, 0.1);
//        sleep(1000);
//        robot.SetPowerDouble4(0, 0, 1, 0, 0.1);
//        sleep(1000);
//        robot.SetPowerDouble4(0, 0, 0, 1, 0.1);
//        sleep(1000);


//        robot.SetPowerDouble4(1.0, 0, 0, 0, 0.1);
//        sleep(2500);
//        robot.SetPowerDouble4(0, 1.0, 0, 0, 0.1);
//        sleep(2500);
//        robot.SetPowerDouble4(0, 0, 1.0, 0, 0.1);
//        sleep(2500);
//        robot.SetPowerDouble4(0, 0, 0, 1.0, 0.1);
//        sleep(2500);

    }
}