package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode", group = "Sensor")
public class TestingOpMode extends LinearOpMode {
    Robot robot = new Robot();


    public ElapsedTime deltaTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();
        double d = robot.GetRotation();
        while (opModeIsActive()) {
            robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.4, 20, 0, 25, 90, d);
            sleep(1000);
            robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.4, 20, 0, 25, -90, d);
            sleep(1000);
        }
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
