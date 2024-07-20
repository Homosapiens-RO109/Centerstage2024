package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class motor_test extends LinearOpMode {
    public DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    double x, y, rt, den, powFrontLeft, powBackLeft, powFrontRight, powBackRight;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFS");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorSS");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFD");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorSD");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a)
                motorBackRight.setPower(1);
            if(gamepad1.b)
                motorBackLeft.setPower(1);
            if(gamepad1.y)
                motorFrontLeft.setPower(1);
            if(gamepad1.x)
                motorFrontRight.setPower(1);
        }
    }
}
