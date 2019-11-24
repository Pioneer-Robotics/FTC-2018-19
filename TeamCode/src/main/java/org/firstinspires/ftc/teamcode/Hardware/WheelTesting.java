package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Wheel Testing", group = "Calibration")
public class WheelTesting extends LinearOpMode {

    Robot robot = new Robot();

    public ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("X", "FL");
            telemetry.update();
            robot.SetPowerDouble4(1, 0, 0, 0, 1);
            sleep(2500);

            telemetry.addData("Y", "FR");
            telemetry.update();
            robot.SetPowerDouble4(0, 1, 0, 0, 1);
            sleep(2500);

            telemetry.addData("Z", "BL");
            telemetry.update();
            robot.SetPowerDouble4(0, 0, 1, 0, 1);
            sleep(2500);


            telemetry.addData("W", "BR");
            telemetry.update();
            robot.SetPowerDouble4(0, 0, 0, 1, 1);
            sleep(2500);

            deltaTime.reset();
        }
    }

}
