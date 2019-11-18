package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bDeltaValue;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Wheel Testing", group = "Debugging")
//Used to make sure all wheels are set up correctly
public class TestingOpMode3 extends LinearOpMode {

    Robot robot = new Robot();

    public bDeltaValue frontLeft;
    public bDeltaValue frontRight;
    public bDeltaValue backLeft;
    public bDeltaValue backRight;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);


        while (!opModeIsActive()) {
            frontLeft.Assign(robot.driveManager.frontLeft.motor.getCurrentPosition());
            frontRight.Assign(robot.driveManager.frontRight.motor.getCurrentPosition());
            backLeft.Assign(robot.driveManager.backLeft.motor.getCurrentPosition());
            backRight.Assign(robot.driveManager.backRight.motor.getCurrentPosition());

            telemetry.addData("Front Left Encoder Delta", frontLeft.delta());
            telemetry.addData("Front Right Encoder Delta", frontRight.delta());
            telemetry.addData("Back Left Encoder Delta", backLeft.delta());
            telemetry.addData("Back Right Encoder Delta", backRight.delta());

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.SetPowerDouble4(1, 1, 1, 1, 0.25);

            frontLeft.Assign(robot.driveManager.frontLeft.motor.getCurrentPosition());
            frontRight.Assign(robot.driveManager.frontRight.motor.getCurrentPosition());
            backLeft.Assign(robot.driveManager.backLeft.motor.getCurrentPosition());
            backRight.Assign(robot.driveManager.backRight.motor.getCurrentPosition());

            telemetry.addData("Front Left Encoder Delta", frontLeft.delta());
            telemetry.addData("Front Right Encoder Delta", frontRight.delta());
            telemetry.addData("Back Left Encoder Delta", backLeft.delta());
            telemetry.addData("Back Right Encoder Delta", backRight.delta());


            telemetry.update();

            sleep(250);

        }

        robot.Stop();
    }

}
