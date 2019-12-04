package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Skystone", group = "ftcPio")
public class SkystoneAuto extends Auto {

    public double startRotation;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();


        waitForStart();

        robot.arm.SetArmState(0.1, 0, 1, 1);

        //If we can't see the skystone, move foward a tad to get a better reading
        while (opModeIsActive()) {
            if (jobs.tensorFlowaJob.getCurrentRecognition() == null) {
                robot.DriveByDistance(speed_low, 2.5);
                sleep(500);
            }
            if (jobs.tensorFlowaJob.getCurrentRecognition() != null) {
                StopMovement();
                break;
            }
        }


        //Line up with a skystone
        //A lockThreshold of .25 will get is within 19.5 degrees of the stone
        SkystoneAlign(speed_low, 45, 2, 0.5, 0.25, startRotation);

        //Drive forward while adjusting heading to line up with the skystone
        DriveAtSkystone(speed_med, 25, 35, startRotation);

        jobs.tensorFlowaJob.Stop();

        robot.arm.SetArmState(0, 0.35, 1, 1);
        robot.arm.SetGripState(1, 0.5);


        sleep(2500);


        robot.arm.SetGripState(0.35, 0.5);


        //Deploy the arm and grab dat stone
        ActuateArm();

        //Roll back a wee bit
        robot.DriveByDistance(speed_med, -5);

        //Rotate based on our side to face the back sensors towards the foundation
        if (side == FieldSide.SIDE_BLUE) {
            robot.RotatePID(startRotation + 90, speed_med, 100000);
        } else {
            robot.RotatePID(startRotation - 90, speed_med, 10000);
        }


        //Move towards the foundation by wall tracking along the wall
        ResetWallPID();
        while (opModeIsActive() && robot.GetDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM) > 100) {
            if (side == FieldSide.SIDE_BLUE) {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group90, speed_med, 36, walltrackingController, 35, -90, robot.GetRotation());
            } else {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group270, speed_med, 36, walltrackingController, 35, 90, robot.GetRotation());

            }

            if (!opModeIsActive()) {
                break;
            }
        }

        StopMovement();

        //Rotate to face the foundation
//            robot.RotatePID(0, speed_high, 10000);


        //Deploy the arm to latch to the foundation
        ActuateArm();

        StopRobot();
    }
}
