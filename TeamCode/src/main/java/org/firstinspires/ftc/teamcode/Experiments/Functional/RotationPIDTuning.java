package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "PIDTuning", group = "Sensor")
public class RotationPIDTuning extends LinearOpMode {

    Robot robot = new Robot();

    Double3 PID = new Double3();

    DeltaTime deltaTime = new DeltaTime();

    TuningMode mode = TuningMode.P;

    enum TuningMode {
        P, I, D
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        //While not started: Use the controller to tune PID
        while (!isStarted()) {
            deltaTime.Start();

            if (gamepad1.y) {
                mode = TuningMode.P;
            }
            if (gamepad1.a) {
                mode = TuningMode.I;
            }
            if (gamepad1.b) {
                mode = TuningMode.D;
            }


            if (mode == TuningMode.P) {
                telemetry.addData("ADJUSTING P ", PID.x);
                PID.x += gamepad1.right_stick_y * deltaTime.deltaTime() * (gamepad1.x ? 10 : 1);
            }
            if (mode == TuningMode.I) {
                telemetry.addData("ADJUSTING I ", PID.y);
                PID.y += gamepad1.right_stick_y * deltaTime.deltaTime() * (gamepad1.x ? 10 : 1);
            }
            if (mode == TuningMode.D) {
                telemetry.addData("ADJUSTING D ", PID.z);
                PID.z += gamepad1.right_stick_y * deltaTime.deltaTime() * (gamepad1.x ? 10 : 1);
            }


            telemetry.addData("P : ", PID.x);
            telemetry.addData("I : ", PID.y);
            telemetry.addData("D : ", PID.z);
            telemetry.update();
            deltaTime.Stop();
        }

        waitForStart();

        while (opModeIsActive()) {
            //Move via joystick and maintain rotation
            robot.SetRotationPID(90, 1, PID.x, PID.y, PID.z);

        }

    }
}