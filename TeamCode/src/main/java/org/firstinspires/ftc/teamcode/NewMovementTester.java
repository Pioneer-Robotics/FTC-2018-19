package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="FTCPio")
public class NewMovementTester extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    Movement mov = new Movement();
    ElapsedTime runtime = new ElapsedTime();
    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        mov.init(robot.motorLeft,robot.motorRight,robot.imu,this,runtime, COUNTS_PER_INCH);
        while (opModeIsActive()) {
            while (!gamepad1.a) {
            }
            mov.experimentalTurn(1,-90);
            while (!gamepad1.a) {
            }
            mov.experimentalDrive(1,100,30);
        }
    }
}
