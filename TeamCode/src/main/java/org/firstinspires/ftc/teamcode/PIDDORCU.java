package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class PIDDORCU extends LinearOpMode {
    PIDController controller;
    FtcDashboard dashboard;
    private DcMotor MotorDr, MotorSt;

    public static double kp = 0.01, ki = 0, kd = 0, kfst, kfdr;

    public static int target = 0;

    public final double ticks_in_degree = 532/360.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MotorDr = hardwareMap.get(DcMotor.class, "motordr");
        MotorSt = hardwareMap.get(DcMotor.class, "motorst");

        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MotorDr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorDr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorDr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorDr.setDirection(DcMotorSimple.Direction.REVERSE);
        target = 0;
        waitForStart();

        while(opModeIsActive()) {
            target += -gamepad1.right_stick_y * 50;
            controller.setPID(kp, ki, kd);
            double power = controller.calculate(MotorSt.getCurrentPosition(), target);
            double power2 = controller.calculate(MotorDr.getCurrentPosition(), target);
            double Kff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
            double Kff2 = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
            MotorSt.setPower(power + Kff);
            MotorDr.setPower(power2 + Kff2);
            
            telemetry.addData("Target: ", target);
            telemetry.addData("Current Position: ", MotorDr.getCurrentPosition());
            telemetry.update();
        }
    }
}
