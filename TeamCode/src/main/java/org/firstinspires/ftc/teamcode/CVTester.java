package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="CVTest", group="FTCPio")
public class CVTester extends LinearOpMode {
    private HardwareInfinity robot = new HardwareInfinity();
    CVManager tFlow = new CVManager();
    CamManager camM = new CamManager();
    int choose = 0;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        tFlow.init(robot.webCam, robot.tFlowId, camM);
        camM.init(robot, tFlow);
        tFlow.disable = true;
        waitForStart();
        tFlow.start();
        camM.start();

        while (!robot.botSwitch.getState() && !robot.topSwitch.getState()) {
            robot.linearArm.setPower(1);
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("St:", "%d", tFlow.st);
            telemetry.addData("Status:", "%d", tFlow.Status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.addData("GO:", tFlow.go ? "True" : "False");
            telemetry.update();
        }
        robot.linearArm.setPower(0);
        sleep(100);
        while (opModeIsActive()) {
            if (isStopRequested()) {
                tFlow.go=false;
                camM.go=false;
                return;
            }
            if (tFlow.Status == 1) {
                choose = (int) tFlow.minDat[0];
            }
            if (tFlow.Status == 2) {

            }
            telemetry.addData("Choose:", "%d", choose);
            telemetry.addData("St:", "%d", tFlow.st);
            telemetry.addData("Status:", "%d", tFlow.Status);
            telemetry.addData("Tar:","%d",tFlow.tar);
            telemetry.addData("MinDat:", "{%.3f, %.3f}",tFlow.minDat[0],tFlow.minDat[1]);
            telemetry.addData("GO:", tFlow.go ? "True" : "False");
            if (gamepad1.a && !tFlow.go) {
                tFlow.go = true;
                tFlow.start();
            }
            telemetry.update();

        }
    }
}
