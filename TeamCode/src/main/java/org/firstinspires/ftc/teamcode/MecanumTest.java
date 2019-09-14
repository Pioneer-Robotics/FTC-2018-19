package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="MecTest", group="FTCPio")
public class MecanumTest extends Teleop {

    double leftDiagPower = 0;
    double rightDiagPower = 0;
    double leftRotatePower = 0;
    double rightRotatePower = 0;
    double angle = 0;
    boolean coordinate_system_lock = true;
    double imu_offset = 0;
    boolean coordinate_system_lock_transition_controller = true;
    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();
    private static final double sq2 = Math.sqrt(2);

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    BNO055IMU imu;

    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IParameters.loggingEnabled = true;
        IParameters.loggingTag = "IMU";
        imu.initialize(IParameters);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    boolean lockToggle = false;

    @Override


    public void loop() {
        if (coordinate_system_lock) {
            angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) - imu_offset;
            leftDiagPower = ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2 * Math.sin(angle) + ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2) * Math.cos(angle));
            rightDiagPower = ((-(-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2) * Math.sin(angle) + ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2 * Math.cos(angle)));
        } else {
            leftDiagPower = ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2);
            rightDiagPower = ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2);
        }
        if (gamepad1.a) {
            if (coordinate_system_lock_transition_controller) {
                coordinate_system_lock = !coordinate_system_lock;
                imu_offset = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            }
            coordinate_system_lock_transition_controller = false;
        } else {
            coordinate_system_lock_transition_controller = true;
        }
        if (gamepad1.b) {


            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        telemetry.addData("Lock: ", coordinate_system_lock);
        telemetry.update();
        leftRotatePower = gamepad1.right_stick_x;
        rightRotatePower = -gamepad1.right_stick_x;
        frontLeft.setPower(leftDiagPower+leftRotatePower);
        frontRight.setPower(rightDiagPower+rightRotatePower);
        backLeft.setPower(rightDiagPower+leftRotatePower);
        backRight.setPower(leftDiagPower+rightRotatePower);
    }
}