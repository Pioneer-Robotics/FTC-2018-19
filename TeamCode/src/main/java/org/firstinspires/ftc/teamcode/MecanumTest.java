package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MecTest", group="FTCPio")
public class MecanumTest extends Teleop {

    double leftDiagPower = 0;
    double rightDiagPower = 0;
    double leftRotatePower = 0;
    double rightRotatePower = 0;
    private static final double sq2 = Math.sqrt(2);

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftDiagPower = (-gamepad1.left_stick_y-gamepad1.left_stick_x)/sq2;
        rightDiagPower = (-gamepad1.left_stick_y+gamepad1.left_stick_x)/sq2;
        leftRotatePower = gamepad1.right_stick_x;
        rightRotatePower = -gamepad1.right_stick_x;
        frontLeft.setPower(leftDiagPower+leftRotatePower);
        frontRight.setPower(rightDiagPower+rightRotatePower);
        backLeft.setPower(rightDiagPower+leftRotatePower);
        backRight.setPower(leftDiagPower+rightRotatePower);
    }
}
