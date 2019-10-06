package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//10.6.19, proof of concept to move the robot to look at a skystone
@Autonomous(name = "WTest", group = "Sensor")
public class TF_LookAtTest extends LinearOpMode {
    HardwareInfinityMec hwInf = new HardwareInfinityMec();

    TF_ThreadTest TF_thread = new TF_ThreadTest();

    @Override
    public void runOpMode() throws InterruptedException {
        hwInf.init(hardwareMap, this);
        TF_thread.startFromOpmode(this);
        TF_thread.start();

        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwInf.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Loopy loop loop that loops
        while (opModeIsActive()) {
            Recognition skystone = TF_thread.skyStone();
            if (skystone != null) {
                float factor = skystone.getRight() - skystone.getImageWidth();
                double rotationDelta = factor / skystone.getImageWidth();

                hwInf.MoveComplex(0, 0, hwInf.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + rotationDelta);
            }

        }

        //Stop the thread or die!
        TF_thread.stopThread();

        //Stop the motors when the robots done doin what its doin.
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
