package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp
public class TestCalin extends LinearOpMode {
    private DcMotor Stsus,Stjos,Drsus,Drjos;
    public double MotorSpeed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Stsus = hardwareMap.get(DcMotor.class, "stsus");
        Stjos = hardwareMap.get(DcMotor.class, "stjos");
        Drsus = hardwareMap.get(DcMotor.class, "drsus");
        Drjos = hardwareMap.get(DcMotor.class, "drjos");

        Drsus.setDirection(DcMotorSimple.Direction.REVERSE);
        Drjos.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        waitForStart();
        if(isStopRequested())
            return;
        while(opModeIsActive()) {
            if(gamepad1.x)
                imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (gamepad1.dpad_up)
                MotorSpeed = 1;
            if (gamepad1.dpad_left)
                MotorSpeed = 0.75;
            if (gamepad1.dpad_right)
                MotorSpeed = 0.5;
            if (gamepad1.dpad_down)
                MotorSpeed = 0.25;


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (-rotY + rotX + rx) / denominator * MotorSpeed;
            double backLeftPower = (-rotY - rotX + rx) / denominator * MotorSpeed;
            double frontRightPower = (-rotY - rotX - rx) / denominator * MotorSpeed;
            double backRightPower = (-rotY + rotX - rx) / denominator * MotorSpeed;

            Stsus.setPower(frontLeftPower);
            Stjos.setPower(backLeftPower);
            Drsus.setPower(frontRightPower);
            Drjos.setPower(backRightPower);
        }
    }
}
