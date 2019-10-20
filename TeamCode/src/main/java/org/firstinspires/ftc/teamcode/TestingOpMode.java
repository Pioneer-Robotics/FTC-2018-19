package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;

@Autonomous(name = "TestingOpMode", group = "Sensor")
public class TestingOpMode extends LinearOpMode {
    HardwareInfinityMec robot = new HardwareInfinityMec();


    public DeltaTime deltaTime = new DeltaTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        //Reset the encoders and them tell em to drive.
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.SetRotationExperimental(90, -5, 1);
//        sleep(1000);
//        robot.SetRotationExperimental(100, 5, 0.25);


        robot.Stop();
        //Stop the motors when the robots done doin what its doin.
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
