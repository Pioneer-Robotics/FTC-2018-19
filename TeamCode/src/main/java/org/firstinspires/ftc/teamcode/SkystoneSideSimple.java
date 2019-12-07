package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Skystone2", group = "ftcPio")
public class SkystoneSideSimple extends Auto {

    public double startRotation;

    @Override
    public void runOpMode() {
        StartRobot();

        waitForStart();

        sleep(1000);

        robot.DriveByDistance(0.25, 25);

        if (side == FieldSide.SIDE_BLUE) {
            robot.MoveComplex(-90, 0.75, robot.GetRotation() - startRotation);
        }
        if (side == FieldSide.SIDE_RED) {
            robot.MoveComplex(90, 0.75, robot.GetRotation() - startRotation);
        }

        sleep(1500);

        StopMovement();
        StopRobot();
    }
}
