package org.firstinspires.ftc.teamcode.arghh;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake{
    final DcMotor motorin;
    final CRServo servoin;
    final Servo servodrop;
    public static int ROTIRE_NORMALA = 1;
    public static int ROTIRE_INVERS = -1;
    public static int SERVO_SUS = 0;
    public static int STOP_ROTIRE = 0;
    public static double SERVO_JOS = 0.7;
    HardwareMap hardwareMap;
    public ElapsedTime timer= new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motorin = hardwareMap.get(DcMotor.class, "min");
        this.servoin = hardwareMap.get(CRServo.class, "sin");
        this.servodrop = hardwareMap.get(Servo.class, "sd");
        init();
    }
    public void init() {
        servodrop.setPosition(SERVO_SUS);
        motorin.setPower(STOP_ROTIRE);
        servoin.setPower(STOP_ROTIRE);
    }
    public void start_in() {
        servodrop.setPosition(SERVO_JOS);
        motorin.setPower(ROTIRE_NORMALA);
        servoin.setPower(ROTIRE_NORMALA);
        timer.reset();
    }
    public void reverse_in() {
        servodrop.setPosition(SERVO_JOS);
        motorin.setPower(ROTIRE_INVERS);
        servoin.setPower(ROTIRE_INVERS);
        timer.reset();
    }
    void stop_in() {
        servodrop.setPosition(SERVO_SUS);
        motorin.setPower(STOP_ROTIRE);
        servoin.setPower(STOP_ROTIRE);
        timer.reset();
    }
}