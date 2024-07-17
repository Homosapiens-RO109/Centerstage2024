package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@TeleOp
public class se_roteste extends LinearOpMode {
    private DcMotor motor;
    ElapsedTime timerIntake = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && motor.getPower() == 0 && timerIntake.time(TimeUnit.SECONDS) > 1) {
                motor.setPower(1);
                timerIntake.reset();

            }

            if (gamepad1.a && motor.getPower() != 0 && timerIntake.time(TimeUnit.SECONDS) > 1) {
                motor.setPower(0);
                timerIntake.reset();
            }
        }
    }
    }

