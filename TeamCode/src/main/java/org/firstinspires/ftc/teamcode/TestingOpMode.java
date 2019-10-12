package org.firstinspires.ftc.teamcode;


import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "TestingOpMode", group = "Sensor")
public class TestingOpMode extends LinearOpMode {
    HardwareInfinityMec hwInf = new HardwareInfinityMec();


    public DeltaTime deltaTime = new DeltaTime();


    @Override
    public void runOpMode() throws InterruptedException {
        hwInf.init(hardwareMap, this);

        //Reset the encoders and them tell em to drive.
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwInf.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hwInf.SetPowerDouble4(new Double4 (1,1,1,1),1);
        sleep(3000);
/*
        hwInf.SetPowerDouble4(new Double4(1,0,0,0), 1);
        sleep(3000);
        hwInf.SetPowerDouble4(new Double4(0,1,0,0), 1);
        sleep(3000);
        hwInf.SetPowerDouble4(new Double4(0,0,1,0), 1);
        sleep(3000);
        hwInf.SetPowerDouble4(new Double4(0,0,0,1), 1);
        sleep(3000);
*/
//        hwInf.MoveSimple(0, 1);
//        sleep(1000);
//        hwInf.MoveSimple(90, 1);
//        sleep(1000);
//        hwInf.MoveSimple(180, 1);
//        sleep(1000);
//        hwInf.MoveSimple(270, 1);
//        sleep(1000);


        //Stop the motors when the robots done doin what its doin.
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
