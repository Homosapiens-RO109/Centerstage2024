package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Config
@TeleOp
public class Parcare extends LinearOpMode {
    DcMotor MotorParcare;
    @Override
    public void runOpMode() throws InterruptedException
    {
        MotorParcare = hardwareMap.get(DcMotor.class,"parcare");
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.x)
                MotorParcare.setPower(0.5);
            if(gamepad1.a)
                MotorParcare.setPower(0);
            if(gamepad1.b)
                MotorParcare.setPower(-0.5);
            telemetry.addData("Pozitie", MotorParcare.getCurrentPosition());
            telemetry.update();
        }
    }
}
