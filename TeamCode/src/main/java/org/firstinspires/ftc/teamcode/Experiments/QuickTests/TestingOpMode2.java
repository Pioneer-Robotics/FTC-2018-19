package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import android.renderscript.Double2;
import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Experiments.Functional.WallTrackTesting;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode2", group = "Sensor")
public class TestingOpMode2 extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        robot.SetPowerDouble4(1, 1, 1, 1, 0.1);



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