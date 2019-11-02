package org.firstinspires.ftc.teamcode.Experiments.Functional;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

//As of 9.15.19.0706 this serves only to test sensor input
//As of 10.1.19 1549 this serves to track along walls smoothly
//TODO: Add curves smoothing to rotation values. Also using a paralabic function for movement smoothing could be cool?
@Autonomous(name = "Sensors", group = "Sensor")
public class SensorTeleop extends LinearOpMode {
    Robot hwInf = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        hwInf.init(hardwareMap, this);

        waitForStart();
        Robot.instance.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group90, 0, 50, 5, 5);

        //Loopy loop loop that loops
        while (opModeIsActive()) {
            Robot.instance.Op.telemetry.addData("45", Robot.instance.wallTrack.currentGroup.distanceSensors[0].getDistance(DistanceUnit.CM));
            Robot.instance.Op.telemetry.addData("90", Robot.instance.wallTrack.currentGroup.distanceSensors[1].getDistance(DistanceUnit.CM));
            Robot.instance.Op.telemetry.addData("135", Robot.instance.wallTrack.currentGroup.distanceSensors[2].getDistance(DistanceUnit.CM));
            Robot.instance.Op.telemetry.update();
        }

    }

}
