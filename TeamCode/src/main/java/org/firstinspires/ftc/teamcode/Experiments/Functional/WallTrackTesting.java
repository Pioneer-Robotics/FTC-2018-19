package org.firstinspires.ftc.teamcode.Experiments.Functional;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareInfinityMec;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

//As of 9.15.19.0706 this serves only to test sensor input
//As of 10.1.19 1549 this serves to track along walls smoothly
//TODO: Add curves smoothing to rotation values. Also using a paralabic function for movement smoothing could be cool?
@Autonomous(name = "WTest", group = "Sensor")
public class WallTrackTesting extends LinearOpMode {
    HardwareInfinityMec hwInf = new HardwareInfinityMec();

    public AvoidanceConfiguration avoidanceConfig = new AvoidanceConfiguration();

    public SensorTriplet sensors = new SensorTriplet();
    public DeltaTime deltaTime = new DeltaTime();

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

        public double getDistanceAverage(DistanceUnit unit) {
            return (getDistance(TripletType.Center, unit) + getDistance(TripletType.Right, unit) + getDistance(TripletType.Left, unit)) / 3;
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(TripletType type) {
            return type == TripletType.Center ? distanceSensors[1] : (type == TripletType.Right ? distanceSensors[2] : distanceSensors[0]);
        }

        public double getWallAngle() {
            return Math.toDegrees(((bMath.pi() * 3) / 4) - Math.atan(getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM) / getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM)));
        }

        //Returns true if the three sensors have hit a perfect (with in 5%, see "error") line, this can be used to check if there's another robot or obstacle near us
        //Get bounds to work on the sensors
        //Error is between 0 (0%) and 1 (100%)
        public boolean isValid(double error) {
            double sinPiOver4 = bMath.sq2() / 2;
            double e = getDistance(TripletType.Left, DistanceUnit.CM) * sinPiOver4;
            double w = getDistance(TripletType.Center, DistanceUnit.CM);
            double q = getDistance(TripletType.Right, DistanceUnit.CM) * sinPiOver4;
            double difference = Math.abs((q - w) / q - (w - e) / e);
            if (difference <= error) {
                return true;
            } else {
                return false;
            }

        }


        //</editor-fold>
    }

    public static class AvoidanceConfiguration {

        public double currentDistance;

        //Target distance to track too
        public double range;

        //Bounds around the range
        //-b+[range]+b
        public double bounds;

        //The amount of degrees to move in order to correct movement, 45 = fast, 0 = no correction, 25 recommended
        public double correctionScale;

        //Init the config with range bounds and scale
        public AvoidanceConfiguration(double _range, double _bounds, double _correctionScale) {
            range = _range;
            bounds = _bounds;
            correctionScale = _correctionScale;
        }

        public AvoidanceConfiguration() {
        }

        //Returns a number between -1 and 1 based on which way we need to move deh bot and how fast we need too
        public Double CorrectionCoefficient() {

            double factor = 0;

            factor = (Math.abs(currentDistance - range) - bounds) / bounds;

            factor = bMath.Clamp(factor, 0, 1);

            //Multiply by -1 if we need to move away from the wall
            factor *= currentDistance > range ? 1 : -1;

            return factor;
        }

        //The angle that we wanna move in (additive)
        public Double targetDirection() {
            return CorrectionCoefficient() * correctionScale;
        }

        //Called to update the current distance var
        public void SetCurrentDistance(double value) {
            currentDistance = value;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        hwInf.init(hardwareMap, this);

        //Reset the encoders and them tell em to drive. Figure out what an encoder is at some point eh?
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwInf.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Init both the sensor set ups and the avoidance configs
        sensors = new SensorTriplet(this, "sensorL", "sensorM", "sensorR");
        avoidanceConfig = new AvoidanceConfiguration(50, 1, 25);


        //Declare var's out of the while loop to avoid that evil GC monster that hides under your desk at night,
        double curDriveAngle = 0;
        double wallAngle = 0;
        double currentAngle = 0;
        double distance = 0;
        double weightedWallAngle = 0;

        weightedWallAngle = Math.toRadians(sensors.getWallAngle() - 90);


        //Loopy loop loop that loops
        while (opModeIsActive()) {
            deltaTime.Start();
            telemetry.addData("Right sensor data : ", sensors.getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM));
            telemetry.addData("Left sensor data : ", sensors.getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM));
            telemetry.addData("Mid sensor data : ", sensors.getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM));

            wallAngle = sensors.getWallAngle();
            weightedWallAngle = bMath.MoveTowardsRadian(weightedWallAngle, Math.toRadians(wallAngle - 90), deltaTime.deltaTime() * 3);

            //Snap to that angle if we are too far from the target angle (90deg)
            //REMOVE THIS AFTER MORE LAZERS
            if (Math.abs(weightedWallAngle - Math.toRadians(wallAngle - 90)) > bMath.pi() / 2) {
                weightedWallAngle = Math.toRadians(wallAngle - 90);
            }


            //currentAngle = hwInf.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Wall angle : ", wallAngle);
            telemetry.addData("Difference in angle : ", (currentAngle - wallAngle));

            telemetry.update();

            distance = sensors.getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM);
            avoidanceConfig.SetCurrentDistance(distance);

            //Move thy self away from nearby walls using a linear smoothed function (try parabalalalas if big bored strikes again?)
            if (distance < 100) {
                curDriveAngle = wallAngle + avoidanceConfig.targetDirection();
            }

            //True == wall track like a cool robot, false == move forward and rotate like the little spastic robot know deep down we are!
            if (false) {
                hwInf.MoveSimple(curDriveAngle, 0.5);
            } else {
                hwInf.MoveComplex(curDriveAngle, 0.5, weightedWallAngle);
//                hwInf.TestNewMovement(0.25, 0.25, 1);
//                hwInf.SetPowerDouble4(new Double4(-1, 1, -1, 1), 0.25);
            }

            deltaTime.Stop();

        }

        //Stop the motors when the robots done doin what its doin.
        hwInf.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
