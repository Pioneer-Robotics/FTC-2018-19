package org.firstinspires.ftc.teamcode.Experiments.Functional;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

import java.util.concurrent.Delayed;
import java.util.concurrent.ThreadPoolExecutor;

//As of 12.3.19 0708 this serves to test the timing of input methods
//As of 9.15.19.0706 this serves only to test sensor input
@Autonomous(name = "Sensors", group = "Sensor")
public class SensorTesting extends LinearOpMode {
    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();


        //Loopy loop loop that loops
        while (opModeIsActive()) {
            deltaTime.reset();
            telemetry.addData("0", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group0).distanceSensors[0].getDistance(DistanceUnit.CM));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();

            telemetry.addData("90", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group90).distanceSensors[0].getDistance(DistanceUnit.CM));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();


            telemetry.addData("180", robot.experimentalInput.getDistance(robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group90).distanceSensors[0]));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();

            telemetry.addData("270", robot.experimentalInput.getDistance(robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group90).distanceSensors[0]));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();

            deltaTime.reset();
            telemetry.addData("0 fast ", robot.experimentalInput.getDistance(robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group0).distanceSensors[0]));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();

            telemetry.addData("90 fast ", robot.experimentalInput.getDistance(robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group90).distanceSensors[0]));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();


            telemetry.addData("180 fast ", robot.experimentalInput.getDistance(robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).distanceSensors[0]));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();

            telemetry.addData("270 fast ", robot.experimentalInput.getDistance(robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group270).distanceSensors[0]));
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());
            deltaTime.reset();

            telemetry.addData("Active threads", Thread.activeCount());
            ;


            telemetry.update();
        }

    }

}
