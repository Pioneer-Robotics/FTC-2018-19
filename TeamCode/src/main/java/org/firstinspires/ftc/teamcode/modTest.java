package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mod",group="FTCPio")
public class modTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        for (int fu = -180;fu<180;fu++) {
            telemetry.addData("Mod:", "%7d",fu%360);
            telemetry.update();
            sleep(100);
        }
    }
}
