package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MovTest", group="FTCPio")
public class NewMovementTester extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    private Movement mov = new Movement();
    private ElapsedTime runtime = new ElapsedTime();
    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        mov.init(robot.motorLeft,robot.motorRight,robot.imu, robot.imu1, runtime, COUNTS_PER_INCH, this);
        waitForStart();
        sleep(100);
        int var = 1;
        while (opModeIsActive() && !isStopRequested()) {
            while (!gamepad1.a) {
                sleep(1);
                if (gamepad1.dpad_down) {
                    if (var==1) mov.pk-=0.05;
                    if (var==2) mov.ik-=0.05;
                    if (var==3) mov.dk-=0.05;
                    telemetry.addData("Variable:", (var == 1) ? "Proportion" : (var == 2) ? "Integral" : "Derivative");
                    telemetry.addData("Proportion:","%.5f",mov.pk);
                    telemetry.addData("Integral:","%.5f",mov.ik);
                    telemetry.addData("Derivative:","%.5f",mov.dk);
                    telemetry.update();
                    sleep(100);
                }
                else if (gamepad1.dpad_up) {
                    if (var==1) mov.pk+=0.05;
                    if (var==2) mov.ik+=0.05;
                    if (var==3) mov.dk+=0.05;
                    telemetry.addData("Variable:", (var == 1) ? "Proportion" : (var == 2) ? "Integral" : "Derivative");
                    telemetry.addData("Proportion:","%.5f",mov.pk);
                    telemetry.addData("Integral:","%.5f",mov.ik);
                    telemetry.addData("Derivative:","%.5f",mov.dk);
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
                    telemetry.addData("Proportion:","%.5f",mov.pk);
                    telemetry.addData("Integral:","%.5f",mov.ik);
                    telemetry.addData("Derivative:","%.5f",mov.dk);
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
                    telemetry.addData("Proportion:","%.5f",mov.pk);
                    telemetry.addData("Integral:","%.5f",mov.ik);
                    telemetry.addData("Derivative:","%.5f",mov.dk);
                    telemetry.update();
                    sleep(200);
                }
                if (isStopRequested()) {
                    return;
                }
            }
            mov.angleTurn(1,-90);
        }
    }
}
