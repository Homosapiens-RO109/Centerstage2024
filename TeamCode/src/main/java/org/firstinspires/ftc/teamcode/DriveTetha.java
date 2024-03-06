package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomie.Hardware;

@Config
@TeleOp
public class DriveTetha extends LinearOpMode {
    public static double kd, ki , kp = 0.004;
    public static int target,Pozitieabeon;
    PIDController controller;
    FtcDashboard dashboard;
    private DcMotor MotorFL,MotorFR,MotorRL,MotorRR, MotorGL, MotorGR, MotorRoata;
    boolean inchis = true;
    double msin,mcos,maxi,thetha,putere,turn,mx,my;
    public static double Servobsjos = 0,Servobssus=0.50,Servobdjos=0.04,Servobdsus=0.54,Servowrinchis=0.85,Servowrdeschis=0.4;
    private Servo servobs,servobd,servowr, servoabeon;
    @Override
    public void runOpMode() throws InterruptedException{

        MotorFL = hardwareMap.get(DcMotor.class, "stsus");
        MotorFR = hardwareMap.get(DcMotor.class, "drsus");
        MotorRL = hardwareMap.get(DcMotor.class, "stjos");
        MotorRR = hardwareMap.get(DcMotor.class, "drjos");
        MotorGR = hardwareMap.get(DcMotor.class, "glis1");
        MotorGL = hardwareMap.get(DcMotor.class, "glis2");
        MotorRoata = hardwareMap.get(DcMotor.class, "m_roata");

        servobd = hardwareMap.get(Servo.class, "Servobd");
        servobs = hardwareMap.get(Servo.class, "Servobs");
        servowr = hardwareMap.get(Servo.class, "Servowr");
        servoabeon = hardwareMap.get(Servo.class, "Avion");

        MotorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRR.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorGR.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRoata.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        servobd.setDirection(Servo.Direction.REVERSE);
        servobd.setPosition(Servobdjos);
        servobs.setPosition(Servobsjos);
        servowr.setPosition(Servowrinchis);
//        servoabeon.setPosition(0);

        MotorGL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorGR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        target = 0;
        waitForStart();
        if(isStopRequested())
            return;
        while(opModeIsActive()) {

            mx = gamepad1.left_stick_x;
            my = gamepad1.left_stick_y;
            turn = gamepad1.right_trigger - gamepad1.left_trigger;

            thetha = Math.atan2(my, mx);
            putere = Math.hypot(mx, my);
            msin = Math.sin(thetha - Math.PI/4);
            mcos = Math.cos(thetha - Math.PI/4);
            maxi = Math.max(Math.abs(msin), Math.abs(mcos));
            double PutereFL = putere * mcos/maxi + turn;
            double PutereFR = putere * msin/maxi - turn;
            double PutereRL = putere * msin/maxi + turn;
            double PutereRR = putere * mcos/maxi - turn;

            if((putere + Math.abs(turn)) > 1)
            {
                PutereFL /= putere + turn;
                PutereFR /= putere + turn;
                PutereRL /= putere + turn;
                PutereRR /= putere + turn;
            }

            MotorFL.setPower(PutereFL);
            MotorFR.setPower(PutereFR);
            MotorRL.setPower(PutereRL);
            MotorRR.setPower(PutereRR);

            

            controller.setPID(kp, ki, kd);
            double KPIDL = controller.calculate(MotorGL.getCurrentPosition(), target);
            double KPIDR = controller.calculate(MotorGR.getCurrentPosition(), target);

            if(MotorGL.getPower() < 0)
                kp = 0.001;
            else kp = 0.004;
            MotorGR.setPower(KPIDR);

            if(gamepad1.dpad_left)
                target = 0;
            if(gamepad1.dpad_up)
                target = 1080;
            if(gamepad1.dpad_right)
                target = 1800;
            if(MotorGL.getCurrentPosition() <= 1200 && target <= 200 && MotorGL.getPower() < 0){
                servobs.setPosition(Servobsjos);
                servobd.setPosition(Servobdjos);
            }
            if(MotorGL.getCurrentPosition() >= 1050 && target >= 1080) {
                servobs.setPosition(Servobssus);
                servobd.setPosition(Servobdsus);
            }
            if(MotorGL.getCurrentPosition() <= 500 && !inchis)
            {
                servowr.setPosition(Servowrinchis);
                inchis = true;
            }

            if(MotorRoata.getPower() != 0)
            {
                if(gamepad1.x || gamepad1.right_bumper)
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

                if(gamepad1.right_bumper)
                {
                    MotorRoata.setPower(-1);
                    sleep(150);
                }
            }

            if(servobs.getPosition() == Servobssus)
            {
                if(gamepad1.b) {
                    servobs.setPosition(Servobsjos);
                    servobd.setPosition(Servobdjos);
                    if(servowr.getPosition() == Servowrdeschis)
                        servowr.setPosition(Servowrinchis);
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

            if(!inchis)
                if(gamepad1.a) {
                    servowr.setPosition(Servowrinchis);
                    inchis = true;
                    sleep(150);
                }

            if(inchis)
                if(gamepad1.a) {
                        servowr.setPosition(Servowrdeschis);
                        inchis = false;
                        sleep(150);
                    }
            if(gamepad1.left_bumper)
            {
                servoabeon.setPosition(Pozitieabeon);
            }

            telemetry.addData("Glisanta stanga", MotorGL.getCurrentPosition());
            telemetry.addData("Glisanta dreapta", MotorGR.getCurrentPosition());
            telemetry.addData("Servo wr pos", servowr.getPosition());
            telemetry.update();
        }
    }
}