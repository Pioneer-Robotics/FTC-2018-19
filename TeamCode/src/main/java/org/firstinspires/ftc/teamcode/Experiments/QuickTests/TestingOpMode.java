package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "TestingOpMode", group = "Sensor")
public class TestingOpMode extends LinearOpMode {
    Robot robot = new Robot();


    public DeltaTime deltaTime = new DeltaTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        robot.RotatePID(90, 0.1f, 1000);

//        while (opModeIsActive()) {
//            sleep(5000);
//
//            robot.SetPowerDouble4(1, 0, 0, 0, 0.25);
//            sleep(1000);
//            robot.SetPowerDouble4(0, 1, 0, 0, 0.25);
//            sleep(1000);
//            robot.SetPowerDouble4(0, 0, 1, 0, 0.25);
//            sleep(1000);
//
//            robot.SetPowerDouble4(0, 0, 0, 1, 0.25);
//            sleep(1000);
//        }

        robot.Stop();
    }

}
