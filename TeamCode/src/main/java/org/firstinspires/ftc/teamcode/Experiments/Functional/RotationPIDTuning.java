package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bDataManger;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "PIDTuning", group = "Sensor")
public class
RotationPIDTuning extends LinearOpMode {

    Robot robot = new Robot();

    Double3 PID = new Double3();

    ElapsedTime deltaTime = new ElapsedTime();

    public bDataManger dataManger = new bDataManger();


    TuningMode mode = TuningMode.P;

    enum TuningMode {
        P, I, D
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        dataManger.Start();
        deltaTime.reset();


        PID.x = dataManger.readData("PID_Testing_P", 3);
        PID.y = dataManger.readData("PID_Testing_I", 0.42);
        PID.z = dataManger.readData("PID_Testing_D", 0.22);

        waitForStart();

        while (opModeIsActive()) {
            //Use the controller to tune PID
            while (!gamepad1.x) {
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
                    PID.x += gamepad1.right_stick_y * -1 * deltaTime.seconds();
                }
                if (mode == TuningMode.I) {
                    telemetry.addData("ADJUSTING I ", PID.y);
                    PID.y += gamepad1.right_stick_y * -1 * deltaTime.seconds();
                }
                if (mode == TuningMode.D) {
                    telemetry.addData("ADJUSTING D ", PID.z);
                    PID.z += gamepad1.right_stick_y * -1 * deltaTime.seconds();
                }
                telemetry.addData("HOW TO : USE  Y TO TUNE P. USE A TO TUNE I. USE B TO TUNE D. HOLD X TO TEST A 90 TURN", "");


                telemetry.addData("P : ", PID.x);
                telemetry.addData("I : ", PID.y);
                telemetry.addData("D : ", PID.z);
                telemetry.addData("deltaTime : ", deltaTime.seconds());
                telemetry.update();
                deltaTime.reset();

            }

            dataManger.writeData("PID_Testing_P", PID.x);
            dataManger.writeData("PID_Testing_I", PID.y);
            dataManger.writeData("PID_Testing_D", PID.z);

            double targetRotation = 90 + robot.GetRotation();
            robot.RotatePID(targetRotation, 1, 10000, PID.x, PID.y, PID.z);
            robot.SetPowerDouble4(0, 0, 0, 0, 0);
        }

        robot.Stop();

    }
}