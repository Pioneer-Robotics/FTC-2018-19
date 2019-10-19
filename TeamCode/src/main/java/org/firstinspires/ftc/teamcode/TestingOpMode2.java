package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestingOpMode2", group = "Sensor")
public class TestingOpMode2 extends LinearOpMode {
    HardwareInfinityMec robot = new HardwareInfinityMec();


    public DeltaTime deltaTime = new DeltaTime();

    public JobsTesting j = new JobsTesting();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        //Reset the encoders and them tell em to drive.
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        j.debuggingJob.Start(this);


        robot.Stop();
        //Stop the motors when the robots done doin what its doin.
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
