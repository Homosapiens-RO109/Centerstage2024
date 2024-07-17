package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;
@TeleOp
public class testmotor extends LinearOpMode{
    private DcMotor motor;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "test_motor");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                motor.setPower(1);
            if (gamepad1.b)
                motor.setPower(0);
        }
    }
}
