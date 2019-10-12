package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//10.11.19: Moves towards any skystone it sees!
@Autonomous(name = "Skystone", group = "Auto Testing")
public class SkystoneAutoTest extends LinearOpMode {

    HardwareInfinityMec robot = new HardwareInfinityMec();

    public TensorFlowThread tensorFlowThread = new TensorFlowThread();

    @Override
    public void runOpMode() {

        //Init the hardware and spawn a TF thread for skystonez
        robot.init(hardwareMap, this);
        tensorFlowThread.startThread(this, "Skystone", 0.5);

        //Set up the movement motors
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double skystoneOffset = 0;
        double forwardMovementFactor = 90;

        //Stop moving once this width is reached by the skystone
        double stopWidth = 250;
        double lerpFactor = 0;

        while (opModeIsActive()) {
            Recognition r = tensorFlowThread.currentRecognition;
            if (r != null) {
//                skystoneOffset = tensorFlowThread.getCurrentXFactor(r) * 45;

                skystoneOffset = bMath.MoveTowards(skystoneOffset, tensorFlowThread.getCurrentXFactor(r) * 45, 0.1);

                lerpFactor = 1 - (stopWidth - tensorFlowThread.getWidth(r)) / stopWidth;
                forwardMovementFactor = bMath.Lerp(90, 0, lerpFactor);


                telemetry.addData("Lerping factor", lerpFactor);
                telemetry.addData("Fwd movement factor", forwardMovementFactor);
                telemetry.addData("Skystone movement offset", tensorFlowThread.getCurrentXFactor(r) * 45);
                telemetry.addData("Skystone mid X factor", tensorFlowThread.getCurrentXFactor(r));
                telemetry.update();
                robot.MoveSimple(90 + tensorFlowThread.getCurrentXFactor(r) * 45, 0.5);

            } else {
                robot.SetPowerDouble4(new Double4(0, 0, 0, 0), 0);

            }


        }


        tensorFlowThread.stopThread();
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
