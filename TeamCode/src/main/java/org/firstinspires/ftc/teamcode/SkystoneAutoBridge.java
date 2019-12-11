package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Skystone2", group = "ftcPio")
public class SkystoneAutoBridge extends Auto {

    public double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    double startDelay = 5;

    double startDistance = 25;

    boolean inverted = false;

    @Override
    public void runOpMode() {
        StartRobot();

        while (!opModeIsActive()) {
            startDelay += gamepad1.left_stick_x * deltaTime.seconds() * 5;
            startDistance += gamepad1.right_stick_x * deltaTime.seconds() * 10;
            if (gamepad1.a) {
                inverted = true;
            }
            if (gamepad1.b) {
                inverted = false;
            }
            telemetry.addData("Start Delay (seconds) ", startDelay);
            telemetry.addData("Drive Distance (CM) ", startDistance);
            telemetry.addData("Drive Inverted (True == Left/Right first. False == drive fwrd first) ", inverted);
            telemetry.update();
            deltaTime.reset();
        }
        waitForStart();

        sleep((long) startDelay * 1000);

        if (inverted) {
            robot.DriveByDistance(0.25, 5);


            if (side == FieldSide.SIDE_BLUE) {
                robot.MoveComplex(-90, 0.75, robot.GetRotation() - startRotation);
            }
            if (side == FieldSide.SIDE_RED) {
                robot.MoveComplex(90, 0.75, robot.GetRotation() - startRotation);
            }

            sleep(1500);

            robot.DriveByDistance(0.25, startDistance - 5);

        } else {

//        robot.DriveByDistance(0.25, 5);
            robot.DriveByDistance(0.25, startDistance);


            if (side == FieldSide.SIDE_BLUE) {
                robot.MoveComplex(-90, 0.75, robot.GetRotation() - startRotation);
            }
            if (side == FieldSide.SIDE_RED) {
                robot.MoveComplex(90, 0.75, robot.GetRotation() - startRotation);
            }

            sleep(1500);

        }
        StopMovement();
        StopRobot();
    }
}
