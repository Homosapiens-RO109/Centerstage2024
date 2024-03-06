package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Test extends LinearOpMode {
    private DcMotor Stsus,Stjos,Drsus,Drjos;
    boolean coi=true;
    @Override
    public void runOpMode() throws InterruptedException {
        Stsus = hardwareMap.get(DcMotor.class, "stsus");
        Stjos = hardwareMap.get(DcMotor.class, "stjos");
        Drsus = hardwareMap.get(DcMotor.class, "drsus");
        Drjos = hardwareMap.get(DcMotor.class, "drjos");

        Drsus.setDirection(DcMotorSimple.Direction.REVERSE);
        Drjos.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if(isStopRequested())
            return;
        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if(gamepad1.b)      {
                coi = true;
            }
            if(gamepad1.a) {
                coi = false;
            }
            if(coi) {
                Stsus.setPower(frontLeftPower);
                Stjos.setPower(backLeftPower);
                Drsus.setPower(frontRightPower);
                Drjos.setPower(backRightPower);
            }
            else {
                Stsus.setPower(frontLeftPower*0.7);
                Stjos.setPower(backLeftPower*0.7);
                Drsus.setPower(frontRightPower*0.7);
                Drjos.setPower(backRightPower*0.7);
            }
        }
    }
}
