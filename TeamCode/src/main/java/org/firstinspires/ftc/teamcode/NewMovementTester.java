package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MovTest", group="FTCPio")
public class NewMovementTester extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        sleep(100);
        int var = 1;
        while (opModeIsActive() && !isStopRequested()) {
            while (!gamepad1.a) {
                sleep(1);
                if (gamepad1.dpad_down) {
                    if (var==1) robot.pk-=0.01;
                    if (var==2) robot.ik-=0.01;
                    if (var==3) robot.dk-=0.01;
                    telemetry.addData("Variable:", (var == 1) ? "Proportion" : (var == 2) ? "Integral" : "Derivative");
                    telemetry.addData("Proportion:","%.5f",robot.pk);
                    telemetry.addData("Integral:","%.5f",robot.ik);
                    telemetry.addData("Derivative:","%.5f",robot.dk);
                    telemetry.update();
                    sleep(100);
                }
                else if (gamepad1.dpad_up) {
                    if (var==1) robot.pk+=0.01;
                    if (var==2) robot.ik+=0.01;
                    if (var==3) robot.dk+=0.01;
                    telemetry.addData("Variable:", (var == 1) ? "Proportion" : (var == 2) ? "Integral" : "Derivative");
                    telemetry.addData("Proportion:","%.5f",robot.pk);
                    telemetry.addData("Integral:","%.5f",robot.ik);
                    telemetry.addData("Derivative:","%.5f",robot.dk);
                    telemetry.update();
                    sleep(100);
                }
                else if (gamepad1.dpad_right) {
                    if (var==1) {
                        var = 2;
                    } else if (var == 2) {
                        var = 3;
                    } else {
                        var = 1;
                    }
                    telemetry.addData("Variable:", (var == 1) ? "Proportion" : (var == 2) ? "Integral" : "Derivative");
                    telemetry.addData("Proportion:","%.5f",robot.pk);
                    telemetry.addData("Integral:","%.5f",robot.ik);
                    telemetry.addData("Derivative:","%.5f",robot.dk);
                    telemetry.update();
                    sleep(200);
                }
                else if (gamepad1.dpad_left) {
                    if (var==1) {
                        var = 3;
                    } else if (var == 2) {
                        var = 1;
                    } else {
                        var = 2;
                    }
                    telemetry.addData("Variable:", (var == 1) ? "Proportion" : (var == 2) ? "Integral" : "Derivative");
                    telemetry.addData("Proportion:","%.5f",robot.pk);
                    telemetry.addData("Integral:","%.5f",robot.ik);
                    telemetry.addData("Derivative:","%.5f",robot.dk);
                    telemetry.update();
                    sleep(200);
                }
                if (isStopRequested()) {
                    return;
                }
            }
            robot.angleTurn(1,-90);
        }
    }
}
