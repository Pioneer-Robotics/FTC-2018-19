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
    double timer;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();
        while (opModeIsActive()) {
            while (timer < 3 && opModeIsActive()) {
                robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.2, 20, 0, 25, 90);
                timer += deltaTime.seconds();
                deltaTime.reset();

                telemetry.addData("timer ", timer);
                telemetry.update();
            }
            timer = 0;
            while (timer < 3 && opModeIsActive()) {
                robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.2, 20, 0, 25, -90);
                timer += deltaTime.seconds();
                deltaTime.reset();

                telemetry.addData("timer ", timer);
                telemetry.update();
            }
            timer = 0;
        }
        robot.Stop();
    }

}
