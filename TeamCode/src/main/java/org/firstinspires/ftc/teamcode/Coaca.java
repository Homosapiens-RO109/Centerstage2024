package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.MalformedParameterizedTypeException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp
public class Coaca extends LinearOpMode {
    public static double kp = 0.001, ki, kd,kfst,kfdr, pos_servoin = 0, pos_servopus = 0.41, servo_error = 0.01,pos_servoluat = 0.20;
    public static int target = 0,timerin,timersvin;
    PIDController controller;
    FtcDashboard dashboard;
    private DcMotor MotorGl, MotorGr, MotorFL, MotorFR, MotorRL, MotorRR, Motorin;
    private Servo ServoL, ServoR, Servoin;
    public boolean ok;
    public final double ticks_in_degree = 532/360.0;

    static double msin, mcos, mmax, x,y, tetha, putere, turn;

    public void runOpMode() throws InterruptedException {
        // test
        MotorFL = hardwareMap.get(DcMotor.class, "stsus");
        MotorFR = hardwareMap.get(DcMotor.class, "drsus");
        MotorRL = hardwareMap.get(DcMotor.class, "stjos");
        MotorRR = hardwareMap.get(DcMotor.class, "drjos");
        Motorin = hardwareMap.get(DcMotor.class, "motorin");
        MotorGl = hardwareMap.get(DcMotor.class, "glis1");
        MotorGr = hardwareMap.get(DcMotor.class, "glis2");

        ServoL = hardwareMap.get(Servo.class, "servost");
        ServoR = hardwareMap.get(Servo.class, "servodr");
        Servoin = hardwareMap.get(Servo.class, "servoin");
        ServoR.setDirection(Servo.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MotorGl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRL.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorGl.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoL.setPosition(pos_servoluat + servo_error);
        ServoR.setPosition(pos_servoluat);
        Servoin.setPosition(0.4);
        ok=true;
        waitForStart();

        while (opModeIsActive()) {
            //hello
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            turn = gamepad1.right_trigger - gamepad1.left_trigger;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
            double PutereFL = (y - x + turn)/denominator;
            double PutereFR = (y + x - turn)/denominator;
            double PutereRL = (y + x + turn)/denominator;
            double PutereRR = (y - x - turn)/denominator;
            // hello
            /*
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -((y - x + rx) / denominator);
            double backLeftPower = -((y + x + rx) / denominator);
            double frontRightPower = -((y + x - rx) / denominator);
            double backRightPower = -((y - x - rx) / denominator);
             */
//            tetha = Math.atan2(y,x);
//            putere = Math.hypot(x,y);
//
//            msin = Math.sin(tetha - Math.PI/4);
//            mcos = Math.cos(tetha - Math.PI/4);
//            mmax = Math.max(Math.abs(msin), Math.abs(mcos));
//
//            double PutereFL = putere * msin/mmax + turn;
//            double PutereFR = putere * mcos/mmax - turn;
//            double PutereRL = putere * mcos/mmax + turn;
//            double PutereRR = putere * msin/mmax - turn;
//
//            if((putere + Math.abs(turn)) > 1)
//            {
//                PutereFL /= putere + turn;
//                PutereFR /= putere + turn;
//                PutereRL /= putere + turn;
//                PutereRR /= putere + turn;
//            }

            MotorFL.setPower(PutereFL * 0.7);
            MotorFR.setPower(PutereFR * 0.7);
            MotorRL.setPower(PutereRL * 0.7);
            MotorRR.setPower(PutereRR * 0.7);


            controller.setPID(kp, ki, kd);
            double power = controller.calculate(MotorGl.getCurrentPosition(), target);
            double power2 = controller.calculate(MotorGr.getCurrentPosition(), target);
            double Kff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
            double Kff2 = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
            MotorGl.setPower(power + Kff);
            MotorGr.setPower(power2 + Kff2);

            if(MotorGl.getCurrentPosition() > -1500 && MotorGl.getPower() > 0)
            {
                ServoL.setPosition(pos_servoluat + servo_error);
                ServoR.setPosition(pos_servoluat);
            }
            if(MotorGl.getCurrentPosition() < -1500 && MotorGl.getPower() < 0)
            {
                ServoL.setPosition(pos_servopus + servo_error);
                ServoR.setPosition(pos_servopus);
            }

            if(Motorin.getPower() == 0) {
                if(timerin > 15) {
                    if (gamepad1.left_bumper) {
                        Motorin.setPower(0.5);
                        timerin = 0;
                    }

                    if (gamepad1.right_bumper) {
                        Motorin.setPower(-0.5);
                        timerin = 0;
                    }
                }
            }

            if(Motorin.getPower() != 0) {
                if((gamepad1.left_bumper || gamepad1.right_bumper) && timerin > 15) {
                    Motorin.setPower(0);
                    timerin = 0;
                }
            }

            if(MotorGl.getCurrentPosition() > -1000 && MotorGl.getPower() > 0 && !ok)
            {
                Servoin.setPosition(0.4);
                ok = true;
            }
            if(MotorGl.getCurrentPosition() < -1000 && MotorGl.getPower() < 0)
            {
                Servoin.setPosition(0);
                ok = false;
            }
            if(gamepad1.a && timersvin > 20)
                if(ok)
                {
                    Servoin.setPosition(0);
                    ok = false;
                    timersvin = 0;
                }
                else {
                    Servoin.setPosition(0.4);
                    ok = true;
                    timersvin = 0;
                }
            timerin++;
            timersvin++;

            if(timerin > 2000)
                timerin = 50;
            if(timersvin > 2000)
                timersvin = 50;

            target += 50 * gamepad1.right_stick_y;

            if(target > 100)
                target = 0;
            if(target < -3000)
                target = -3000;

            if(gamepad1.right_stick_y == 0)
                target = MotorGl.getCurrentPosition();

            telemetry.addData("ServoSt", ServoL.getPosition());
            telemetry.addData("ServoDr", ServoR.getPosition());
            telemetry.addData("Glis1", MotorGl.getCurrentPosition());
            telemetry.addData("Glis2", MotorGr.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("y pow", MotorGl.getPower());
            telemetry.addData("timer", timerin);
            telemetry.addData("timer servo intake", timersvin);
            telemetry.update();
        }
    }
}
