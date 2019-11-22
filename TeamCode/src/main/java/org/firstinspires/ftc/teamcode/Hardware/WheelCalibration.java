package org.firstinspires.ftc.teamcode.Hardware;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Wheel Calibration", group = "Calibration")
public class WheelCalibration extends LinearOpMode {

    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initCalibration(hardwareMap, this);
        start();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Calibration completed in " + getRuntime() + " seconds, please press start to exit.", "");
            telemetry.update();
        }

        robot.Stop();
    }
}