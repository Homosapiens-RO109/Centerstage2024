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
public class  DriveRobotBased extends LinearOpMode {

    private PIDController controller;
    public static double Kp = 0.001, Ki = 0, Kd = 0;
    //0.0001
    public FtcDashboard dashboard;
    private DcMotor MotorFL, MotorFR, MotorRL, MotorRR, MotorRoata;
    private DcMotorEx MotorGL, MotorGR;
    private final double ticks_in_degree = 312 / 180.0;
    public static int target = 0;
    public static int Gvelo = 200;
    boolean vasile = true;
    private Servo servobd, servobs, servowr;
    public static double Lpos = 0.13, Rpos = 0.1, WPos = 0.45;

    public static boolean open_wrist = false;
    public static boolean inpos = true, tabla = false, target_init = false;
    public static double Servobsjos,Servobssus,Servobdjos,Servobdsus,Servowrinchis,Servowrdeschis;
    public class Glis{
        final int nivel1 = 10,nivel2= 1080,nivel3 = 1800;
    }


    @Override
    public void runOpMode() throws InterruptedException {


        MotorFL = hardwareMap.get(DcMotor.class, "stsus");
        MotorFR = hardwareMap.get(DcMotor.class, "drsus");
        MotorRL = hardwareMap.get(DcMotor.class, "stjos");
        MotorRR = hardwareMap.get(DcMotor.class, "drjos");
        MotorGL = hardwareMap.get(DcMotorEx.class, "glis1");
        MotorGR = hardwareMap.get(DcMotorEx.class, "glis2");

        MotorRoata = hardwareMap.get(DcMotor.class, "m_roata");

        servobd = hardwareMap.get(Servo.class, "Servobd");
        servobs = hardwareMap.get(Servo.class, "Servobs");
        servowr = hardwareMap.get(Servo.class, "Servowr");
        servobd.setDirection(Servo.Direction.REVERSE);

        MotorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRR.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorGL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRoata.setDirection(DcMotorSimple.Direction.REVERSE);
// sunteti prajiti
        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(Kp, Ki, Kd);
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        servowr.setPosition(0.45);

        MotorGL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorGR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servobd.setPosition(0.12);
        servobs.setPosition(0.12);

        tabla = false;

        inpos = true;

        target = 0;

        target_init = false;


        waitForStart();
//
        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double PutereFL = (y + x + rx) / denominator;
            double PutereRL = (y - x + rx) / denominator;
            double PutereFR = (y - x - rx) / denominator;
            double PutereRR = (y + x - rx) / denominator;


            MotorFL.setPower(PutereFL * 0.8);
            MotorFR.setPower(PutereFR * 0.8);
            MotorRL.setPower(PutereRL * 0.8);
            MotorRR.setPower(PutereRR * 0.8);

            controller.setPID(Kp, Ki, Kd);
            int glisposl = MotorGL.getCurrentPosition();
            int glisposr = MotorGR.getCurrentPosition();
            double KPIDL = controller.calculate(glisposl, target);
            double KPIDR = controller.calculate(glisposr, target);

            double pow_pidl = KPIDL;
            double pow_pidr = KPIDR;

            if(MotorGL.getCurrentPosition() == target - 10 && MotorGL.getCurrentPosition() <= target + 10)
                MotorGL.setPower(0);
            else MotorGL.setPower(KPIDL);
            if(MotorGR.getCurrentPosition() >= target - 10 && MotorGR.getCurrentPosition() <= target + 10)
                MotorGR.setPower(0);
            else MotorGR.setPower(KPIDR);

            if(gamepad1.dpad_left)
                target = 10;
            if(gamepad1.dpad_up)
                target = 1080;
            if(gamepad1.dpad_right)
                target = 1800;
            if(MotorRoata.getPower() > 0)
            {
                if(gamepad1.x)
                {
                    MotorRoata.setPower(0);
                    sleep(150);
                }

            }
            else
            {
                if(gamepad1.x)
                {
                    MotorRoata.setPower(1);
                    sleep(150);
                }

            }

            if(servobs.getPosition() == Servobsjos)
            {
                if(gamepad1.b) {
                    servobs.setPosition(Servobssus);
                    servobd.setPosition(Servobdsus);
                    sleep(150);
                }
            }
            else if(servobs.getPosition() == Servobsjos)
            {
                if(gamepad1.b)
                {
                    servobs.setPosition(Servobssus);
                    servobd.setPosition(Servobdsus);
                    sleep(150);
                }
            }
            if(servowr.getPosition() == Servowrinchis)
            {
                if(gamepad1.a)
                {
                    servowr.setPosition(Servowrdeschis);
                    sleep(150);
                }
            }
            else
            {
                if(gamepad1.a)
                {
                    servowr.setPosition(Servowrinchis);
                    sleep(150);
                }
            }
            telemetry.addData("Coc",MotorGL.getCurrentPosition());
            telemetry.addData("Cocmaisex",MotorGR.getCurrentPosition());
            telemetry.update();
        }
    }
}

