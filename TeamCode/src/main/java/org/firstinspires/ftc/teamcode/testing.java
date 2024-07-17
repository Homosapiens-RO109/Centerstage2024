package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

@TeleOp
public class testing extends LinearOpMode {
    private DcMotor motorFataStanga, motorFataDreapta, motorSpateStanga, motorSpateDreapta;
    private Servo cleste;
    private double poz_initial = 0, poz_final = 0.4, max;
    @Override
    public void runOpMode() {
        motorFataStanga = hardwareMap.get(DcMotor.class, "FataSt");
        motorFataDreapta = hardwareMap.get(DcMotor.class, "FataDr");
        motorSpateStanga = hardwareMap.get(DcMotor.class, "SpateSt");
        motorSpateDreapta = hardwareMap.get(DcMotor.class, "SpateDr");
        cleste = hardwareMap.get(Servo.class, "Cleste");

        motorFataStanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSpateStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFataDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        cleste.setPosition(poz_initial);


        waitForStart();
        while(opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;


            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            motorFataStanga.setPower(leftFrontPower);
            motorFataDreapta.setPower(rightFrontPower);
            motorSpateStanga.setPower(leftBackPower);
            motorSpateDreapta.setPower(rightBackPower);


            if(gamepad1.right_trigger > 0)
                cleste.setPosition(poz_final);

            if(gamepad1.left_trigger > 0)
                cleste.setPosition(poz_initial);
        }
    }
}
