package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="CVTest", group="FTCPio")
public class CVTester extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    CVManager tFlow = new CVManager();
    ElapsedTime runtime = new ElapsedTime();
    int choose = 0;
    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        tFlow.init(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        waitForStart();
        tFlow.start();

        while (!robot.botSwitch.getState() && !robot.topSwitch.getState() && runtime.milliseconds() < 3000) {
            robot.linearArm.setPower(1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.update();
        }
        robot.linearArm.setPower(0);
        sleep(100);
        while (opModeIsActive() && !isStopRequested()) {
            if (isStopRequested()) {
                return;
            }
            choose = tFlow.Status;
            if (tFlow.Status == -3) {
                if (tFlow.mineralX<233) {
                    choose = 1;
                } else if (tFlow.mineralX<466) {
                    choose = 2;
                } else if (tFlow.mineralX!=0) {
                    choose = 3;
                } else {
                    choose = -4;
                }
            }
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("Status:", "%d", tFlow.Status);
            telemetry.addData("MineralX:", "%.5f", tFlow.mineralX);
            telemetry.addData("GO:", tFlow.go ? "True" : "False");
            if (gamepad1.a && !tFlow.go) {
                tFlow.go = true;
                tFlow.start();
            }
            telemetry.update();

        }
    }
}
