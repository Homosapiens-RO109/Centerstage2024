package org.firstinspires.ftc.teamcode.arghh;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {
    final Servo servoSt;
    final Servo servoDr;
    final Servo servoWrist;
    final Servo servoDegetica;
    final DcMotor motorst;
    final DcMotor motordr;
    public static double error = 0;
    public static double kp = 0.001;
    public static double ki;
    public static double kd;
    public static double kfst;
    public static double kfdr;
    public static double SERVO_IN = 0.21;
    public static double SERVO_OUT = 0.9;
    public static double WRIST_IN = 0.5;
    public static double WRIST_OUT = 0.1;

    public static int target;
    public static int MAX = 3000;
    public static int MIN = 0;
    public static int LIMIT = 1000;

    public final double ticks_in_degree = 532/360.0;
    HardwareMap hardwareMap;
    FtcDashboard dashboard;
    PIDController pid;

    public Outtake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motorst = hardwareMap.get(DcMotor.class, "glisst");
        this.motordr = hardwareMap.get(DcMotor.class, "glisdr");
        motordr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motordr.setDirection(DcMotorSimple.Direction.REVERSE);
        target = 0;

        this.servoSt =  hardwareMap.get(Servo.class,"sst");
        this.servoDr = hardwareMap.get(Servo.class, "sdr");
        this.servoWrist = hardwareMap.get(Servo.class, "swr");
        this.servoDegetica = hardwareMap.get(Servo.class, "sdg");
        servoDr.setDirection(Servo.Direction.REVERSE);
        servoWrist.setDirection(Servo.Direction.REVERSE);
        pid = new PIDController(kp, ki, kd);
        dashboard = FtcDashboard.getInstance();
        init();
    }
    public void init() {
        servoSt.setPosition(SERVO_IN);
        servoDr.setPosition(SERVO_IN+error);
        servoWrist.setPosition(WRIST_IN);
    }
    public boolean slider_peste_lim() {
        return target >= LIMIT;
    }
    public void brat_in() {
        servoSt.setPosition(SERVO_IN);
        servoDr.setPosition(SERVO_IN+error);
        servoWrist.setPosition(WRIST_IN);
    }
    public void brat_out() {
        servoSt.setPosition(SERVO_OUT);
        servoDr.setPosition(SERVO_OUT+error);
        servoWrist.setPosition(WRIST_OUT);
    }
    public void poz_1() {
        target = 0;
    }
    public void poz_2() {
        target = 1100;
    }
    public void poz_3() {
        target = 2000;
    }
    public void poz_4() {
        target = 2900;
    }
    public void PID() {
        pid.setPID(kp, ki, kd);
        double powerLeftSlider = pid.calculate(motorst.getCurrentPosition(), target);
        double powerRightSlider = pid.calculate(motordr.getCurrentPosition(), target);
        double kfLeftSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
        double kfRighSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
        motorst.setPower(powerLeftSlider + kfLeftSlider);
        motordr.setPower(powerRightSlider + kfRighSlider);
    }
}