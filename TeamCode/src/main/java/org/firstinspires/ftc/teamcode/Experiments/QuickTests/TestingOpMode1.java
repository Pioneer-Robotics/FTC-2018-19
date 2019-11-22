package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode1", group = "Sensor")
public class TestingOpMode1 extends LinearOpMode {

    Robot robot = new Robot();

    public ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("X", "FL");
            telemetry.update();
            robot.SetPowerDouble4(1, 0, 0, 0, 1);
            sleep(2500);

            telemetry.addData("Y", "FR");
            telemetry.update();
            robot.SetPowerDouble4(0, 1, 0, 0, 1);
            sleep(2500);

            telemetry.addData("Z", "BL");
            telemetry.update();
            robot.SetPowerDouble4(0, 0, 1, 0, 1);
            sleep(2500);


            telemetry.addData("W", "BR");
            telemetry.update();
            robot.SetPowerDouble4(0, 0, 0, 1, 1);
            sleep(2500);

            deltaTime.reset();
        }
    }

}
