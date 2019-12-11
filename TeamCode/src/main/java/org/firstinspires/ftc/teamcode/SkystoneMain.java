package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone2", group = "ftcPio")
public class SkystoneMain extends Auto {

    public double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();

        waitForStart();

        CycleArm(1);




        StopMovement();
        StopRobot();
    }
}
