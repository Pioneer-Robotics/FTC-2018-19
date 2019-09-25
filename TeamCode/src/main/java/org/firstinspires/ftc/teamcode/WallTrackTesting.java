package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//As of 9.15.19.0706 this serves only to test sensor input

@Autonomous(name = "WTest", group = "Sensor")
public class WallTrackTesting extends LinearOpMode {
    HardwareInfinity1 hwInf = new HardwareInfinity1();

    public SensorTriplet sensors = new SensorTriplet();

    public static class SensorTriplet {

        DistanceSensor[] distanceSensors = new DistanceSensor[3];

        enum TripletType {
            Right,
            Center,
            Left
        }

        //<editor-fold desc="Init setups">
        //dL = the left most sensor
        //dC = the center sensor
        //dR = the right most sensor
        public SensorTriplet(DistanceSensor dL, DistanceSensor dC, DistanceSensor dR) {
            distanceSensors[0] = dL;
            distanceSensors[1] = dC;
            distanceSensors[2] = dR;
        }

        public SensorTriplet(OpMode opMode, String dL, String dC, String dR) {
            distanceSensors[0] = opMode.hardwareMap.get(DistanceSensor.class, dL);
            distanceSensors[1] = opMode.hardwareMap.get(DistanceSensor.class, dC);
            distanceSensors[2] = opMode.hardwareMap.get(DistanceSensor.class, dR);
        }

        public SensorTriplet() {
        }
        //</editor-fold>

        //<editor-fold desc="External return groups">
        public double getDistance(TripletType type, DistanceUnit unit) {
            return sensor(type).getDistance(unit);
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(TripletType type) {
            return type == TripletType.Center ? distanceSensors[1] : (type == TripletType.Right ? distanceSensors[2] : distanceSensors[0]);
        }

        public double getWallAngle() {
            return Math.toDegrees(((bMath.pi() * 3) / 4) - Math.atan(getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM) / getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM)));
        }

        //</editor-fold>
    }


    @Override
    public void runOpMode() throws InterruptedException {
        hwInf.init(hardwareMap, this);
        //Reset the encoders and them tell em to drive. Figure out what an encoder is at some point eh?
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwInf.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        sensors = new SensorTriplet(this, "sensorL", "sensorM", "sensorR");

        //Declare var's out of the while loop to avoid that evil GC monster that hides under your desk at night,
        double curDriveAngle = 0;
        double wallAngle = 0;
        double currentAngle = 0;
        double distance = 0;

        while (opModeIsActive()) {


            telemetry.addData("Right sensor data : ", sensors.getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM));
            telemetry.addData("Left sensor data : ", sensors.getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM));
            telemetry.addData("Mid sensor data : ", sensors.getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM));

            wallAngle = sensors.getWallAngle();
//            currentAngle = hwInf.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Wall angle : ", wallAngle);
            telemetry.addData("Difference in angle : ", (currentAngle - wallAngle));
            telemetry.update();
            distance = sensors.getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM);

            if (distance < 100) {
                curDriveAngle = wallAngle + (distance < 50 ? -10 : (distance > 45 ? 10 : 0));
            }
            //True == move along the wall within 40 - 60 centimeters, false == move forward and rotate like the little spastic robot know we are!
            if (false) {
                hwInf.TestNewMovement(curDriveAngle, 0, 0.5);
            } else {
                hwInf.TestNewMovement(0.25, 0.25, 1);
//                hwInf.SetPowerDouble4(new Double4(-1, 1, -1, 1), 1);
            }
        }
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
}
