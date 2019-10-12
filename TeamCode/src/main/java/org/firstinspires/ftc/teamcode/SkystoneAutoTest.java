package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double skystoneOffset = 0;

        while (opModeIsActive()) {
            if (tensorFlowThread.hasRecognition()) {
                skystoneOffset = bMath.MoveTowards(skystoneOffset,tensorFlowThread.getCurrentXFactor() * 45,0.1);
            }
            robot.MoveSimple(90 + skystoneOffset, 0.5);

        }


        tensorFlowThread.stopThread();
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
