package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class monster_truck extends LinearOpMode {
    DcMotor motorFrontLeft = null, motorBackLeft = null, motorFrontRight = null, motorBackRight = null;
    double x, y, rt, den, powFrontLeft, powBackLeft, powFrontRight, powBackRight;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            powFrontLeft = (y);
            powBackLeft = (y);
            powFrontRight = (y);
            powBackRight = (y);

            motorFrontLeft.setPower(powFrontLeft);
            motorFrontRight.setPower(powFrontRight);
            motorBackLeft.setPower(powBackLeft);
            motorBackRight.setPower(powBackRight);
        }
    }
}
