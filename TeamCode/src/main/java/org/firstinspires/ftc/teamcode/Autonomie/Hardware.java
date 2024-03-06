package org.firstinspires.ftc.teamcode.Autonomie;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
public class Hardware {
    public DcMotor motorFL = null, motorFR = null, motorRL = null, motorRR = null, motorRoata = null, motorGL = null, motorGR = null;
    public Servo servoBD = null, servoBS = null, servoWR = null;
    VoltageSensor VoltageSensor0;
    double MotorSpeed;
    double Raport;
    double VoltajActual;
    double VoltajDorit;
    private final double ticks_in_degree = 312 / 180.0;
    PIDController controller;
    public static double Kp = 0.0005,Ki = 0, Kd = 0.00022, Kf = 0.045;
    public static int target;

    HardwareMap hwmap = null;
    public void init(HardwareMap hard)
    {
        hwmap = hard;
        motorFL = hwmap.dcMotor.get("stsus");
        motorRL = hwmap.dcMotor.get("stjos");
        motorFR = hwmap.dcMotor.get("drsus");
        motorRR = hwmap.dcMotor.get("drjos");
        motorGL = hwmap.dcMotor.get("glis1");
        motorGR = hwmap.dcMotor.get("glis2");
        motorRoata = hwmap.dcMotor.get("motorin");
        servoBD = hwmap.get(Servo.class, "Servodr");
        servoBS = hwmap.get(Servo.class, "Servost");
        servoWR = hwmap.get(Servo.class, "Servoin");
        VoltageSensor0 = hwmap.voltageSensor.iterator().next();

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorbrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorbrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
    }
}
