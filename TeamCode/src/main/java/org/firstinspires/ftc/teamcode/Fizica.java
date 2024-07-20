package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

@TeleOp
public class Fizica extends LinearOpMode {
    DcMotor motorFL, motorRR, motorRL, motorFR, motorRoata;
    double x, y, rt, powFL, powFR, powRL, powRR, den;
    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorRL = hardwareMap.get(DcMotor.class, "motorRL");
        motorRR = hardwareMap.get(DcMotor.class, "motorRR");
        motorRoata = hardwareMap.get(DcMotor.class, "motorRoata");
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rt = gamepad1.right_trigger - gamepad1.left_trigger;
            den = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rt), 1);

            powFL = (y + x + rt) / den;
            powFR = (y - x - rt) / den;
            powRL = (y - x + rt) / den;
            powRR = (y + x - rt) / den;

            motorFL.setPower(powFL);
            motorFR.setPower(powFR);
            motorRL.setPower(powRL);
            motorRR.setPower(powRR);

            if(gamepad1.a)
                motorRoata.setPower(1);

            if(!gamepad1.a)
                motorRoata.setPower(0);
        }
    }
}
