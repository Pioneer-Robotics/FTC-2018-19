package org.firstinspires.ftc.teamcode.Hardware;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Wheel Calibration", group = "Calibration")
public class WheelCalibration extends LinearOpMode {

    Robot robot = new Robot();

    DeltaTime deltaTime = new DeltaTime();

    int last_bl;

    int last_br;

    double targetRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initCalibration(hardwareMap, this);
        robot.Stop();
    }
}