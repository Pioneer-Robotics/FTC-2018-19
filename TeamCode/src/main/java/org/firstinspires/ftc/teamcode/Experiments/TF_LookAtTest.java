package org.firstinspires.ftc.teamcode.Experiments;


import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareInfinityMec;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

//10.6.19, proof of concept to move the robot to look at a skystone
@Autonomous(name = "LookAtDatSkyStone", group = "Sensor")
public class TF_LookAtTest extends LinearOpMode {
    HardwareInfinityMec hwInf = new HardwareInfinityMec();

    TF_ThreadTest TF_thread = new TF_ThreadTest();

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry.addData(":", "STARTING");

        hwInf.init(hardwareMap, this);
//        TF_thread.startFromOpmode(this);
//        TF_thread.start();

        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwInf.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rotationDelta = 0;

        //Loopy loop loop that loops
        while (opModeIsActive()) {
//            hwInf.SetPowerDouble4(new Double4(-1, -1, 1, 1), 1);

            hwInf.MoveComplex(90, 1, 0);
            telemetry.addData("Velocity ", hwInf.imu.getVelocity());
            telemetry.update();
            Recognition skystone = TF_thread.skyStone();
            if (skystone != null) {
                telemetry.addData("Found skystone!", "TRUE");
                float factor = skystone.getRight() - (skystone.getImageWidth() / 2);
                telemetry.addData("Float factor found : ", factor);
                rotationDelta = bMath.MoveTowards(rotationDelta, factor / skystone.getImageWidth(), 1);
                telemetry.addData("Rotation factor found : ", rotationDelta);

                hwInf.SetPowerDouble4(new Double4(rotationDelta, -rotationDelta, rotationDelta, -rotationDelta), 1);


                hwInf.MoveComplex(0, 0, hwInf.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + rotationDelta);
                telemetry.update();
            }

        }

        //Stop the thread or die!
        TF_thread.stopThread();

        //Stop the motors when the robots done doin what its doin.
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
