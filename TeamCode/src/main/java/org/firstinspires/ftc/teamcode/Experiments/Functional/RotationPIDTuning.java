package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "PIDTuning", group = "Sensor")
public class
RotationPIDTuning extends LinearOpMode {

    Robot robot = new Robot();

    Double3 PID = new Double3();

    ElapsedTime deltaTime = new ElapsedTime();

    TuningMode mode = TuningMode.P;

    enum TuningMode {
        P, I, D
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        deltaTime.reset();

        //While not started: Use the controller to tune PID
        while (!opModeIsActive()) {


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
                PID.x += gamepad1.right_stick_y * 1 * deltaTime.seconds();
            }
            if (mode == TuningMode.I) {
                telemetry.addData("ADJUSTING I ", PID.y);
                PID.y += gamepad1.right_stick_y * 1 * deltaTime.seconds();
            }
            if (mode == TuningMode.D) {
                telemetry.addData("ADJUSTING D ", PID.z);
                PID.z += gamepad1.right_stick_y * 1 * deltaTime.seconds();
            }
            telemetry.addData("HOW TO : USE  Y TO TUNE P. USE A TO TUNE I. USE B TO TUNE D. PRESS X TO FINISH", "");


            telemetry.addData("P : ", PID.x);
            telemetry.addData("I : ", PID.y);
            telemetry.addData("D : ", PID.z);
            telemetry.addData("deltaTime : ", deltaTime.seconds());
            telemetry.update();
            deltaTime.reset();

        }

        while (opModeIsActive()) {
            //Move via joystick and maintain rotation
            robot.RotatePID(90, 1, 10000, PID.x, PID.y, PID.z);

        }

    }
}