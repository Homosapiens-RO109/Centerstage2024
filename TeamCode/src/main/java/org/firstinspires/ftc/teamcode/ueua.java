package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
@TeleOp
public class ueua extends LinearOpMode {
    private PIDController controller;
    public FtcDashboard dashboard;
    private DcMotor MotorFL,MotorFR,MotorRL,MotorRR;
    private DcMotorEx MotorGL,MotorGR;
    private final double ticks_in_degree = 312 / 180.0;
    public static int target = 0;
    public static int Gvelo = 200;
    @Override
    public void runOpMode() throws InterruptedException{
        MotorFL = hardwareMap.get(DcMotor.class, "FataStanga");
        MotorFR = hardwareMap.get(DcMotor.class, "FataDreapta");
        MotorRL = hardwareMap.get(DcMotor.class, "SpateStanga");
        MotorRR = hardwareMap.get(DcMotor.class, "SpateDreapta");
//        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
//        MotorRL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if(isStopRequested())
            return;
        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rt = gamepad1.right_trigger - gamepad1.left_trigger;
            double den = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rt), 1);

            double powFL = (y + x + rt) / den;
            double powFR = (y - x - rt) / den;
            double powRL = (y - x + rt) / den;
            double powRR = (y + x - rt) / den;

            double PutereFL = (y + x + rt) / den;
            double PutereRL = (y - x + rt) / den;
            double PutereFR = (y - x - rt) / den;
            double PutereRR = (y + x - rt) / den;

            MotorFL.setPower(PutereFL);
            MotorRL.setPower(PutereRL);
            MotorFR.setPower(PutereFR);
            MotorRR.setPower(PutereRR);

        }
    }
}
