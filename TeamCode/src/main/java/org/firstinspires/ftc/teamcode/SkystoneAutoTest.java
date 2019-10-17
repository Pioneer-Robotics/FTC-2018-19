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

    public TensorFlow_bThread tensorFlowThread = new TensorFlow_bThread();

    public JobsTesting jobs = new JobsTesting();

    @Override
    public void runOpMode() {
        //This should do pretty much everything we need to line up with a sky stone
        jobs.findSkystoneJob.Start(this);
    }
}
