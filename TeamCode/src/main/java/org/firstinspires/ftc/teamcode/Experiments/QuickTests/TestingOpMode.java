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

    double timer = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();
        double d = robot.GetRotation();
        while (opModeIsActive()) {
            while (timer < 3 && opModeIsActive()) {
                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.2, 20, 1, 45, -90, d);
                timer += deltaTime.seconds();
                deltaTime.reset();

                telemetry.addData("timer ", timer);
                telemetry.update();
            }
            timer = 0;
            while (timer < 3 && opModeIsActive()) {
                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.2, 20, 1, 45, 90, d);
                timer += deltaTime.seconds();
                deltaTime.reset();

                telemetry.addData("timer ", timer);
                telemetry.update();
            }
            timer = 0;


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
