package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Test", group="Test")
public class AutoTest extends LinearOpMode {
    HardwareInfinity1 hwInf = new HardwareInfinity1();

    @Override
    public void runOpMode() throws InterruptedException {
        hwInf.init(hardwareMap, this);
        waitForStart();
        hwInf.MecDrive(100, 60,0.5);
        hwInf.MecDrive(100, -60, 0.5);
        hwInf.MecDrive(100,180, 0.5);
    }
}
