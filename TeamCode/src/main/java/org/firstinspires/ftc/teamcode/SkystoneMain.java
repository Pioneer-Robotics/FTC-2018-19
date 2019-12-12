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

        GrabArm(0.35, 0.35);

        if (side == FieldSide.SIDE_BLUE) {
            robot.RotateSimple(-90, 1, 5, 0.2);
//            robot.RotatePID(-90, 1, 100);
        } else {
            robot.RotateSimple(90, 1, 5, 0.2);

//            robot.RotatePID(90, 1, 100);
        }

        DepositeArm(0.35, 1);

        robot.RotateSimple(0, 1, 5, 0.2);


        StopMovement();
        StopRobot();
    }
}
