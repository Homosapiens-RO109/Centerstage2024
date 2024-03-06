package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.lang.annotation.Target;

@Autonomous(name = "DriveForward1Sec", group = "Autonomous")
public class DriveAuto extends LinearOpMode
{
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotorEx motorbrat;
    private Servo servo;
    private VoltageSensor VoltageSensor0;
    double MotorSpeed;
    double Raport;
    double VoltajActual;
    double VoltajDorit;
    private final double ticks_in_degree = 312 / 180.0;
    private PIDController controller;
    public static double Kp = 0.0005,Ki = 0, Kd = 0.00022, Kf = 0.045;
    public static int target;

    @Override
    public void runOpMode()
    {

        servo = hardwareMap.get(Servo.class, "servo");
        motor1 = hardwareMap.dcMotor.get("stangasus");
        motor2 = hardwareMap.dcMotor.get("stangajos");
        motor3 = hardwareMap.dcMotor.get("dreaptasus");
        motor4 = hardwareMap.dcMotor.get("dreaptajos");
        motorbrat = hardwareMap.get(DcMotorEx.class, "motorbrat");
        VoltageSensor0 = hardwareMap.voltageSensor.iterator().next();

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        motorbrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo.setPosition(0.97);
        UpdatePid();
        target = 20;

        waitForStart();

        UpdateVoltage();
        UpdatePid();

        DriveBackwards(1050);
        UpdatePid();
        DoNothing(1000);
        UpdateVoltage();
        DriveRight(450);
        DoNothing(1000);
        servo.setPosition(0.50);


    }
    private void DriveForward(long miliseconds)
    {
        motor1.setPower(MotorSpeed);
        motor2.setPower(MotorSpeed);
        motor3.setPower(MotorSpeed);
        motor4.setPower(MotorSpeed);
        sleep(miliseconds);
    }
    private void DriveBackwards(long miliseconds)
    {
        motor1.setPower(-MotorSpeed);
        motor2.setPower(-MotorSpeed);
        motor3.setPower(-MotorSpeed);
        motor4.setPower(-MotorSpeed);
        sleep(miliseconds);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    private void DriveLeft(long miliseconds)
    {
        motor1.setPower(-MotorSpeed);
        motor2.setPower(MotorSpeed);
        motor3.setPower(-MotorSpeed);
        motor4.setPower(MotorSpeed);
        sleep(miliseconds);
    }
    private void DriveRight(long miliseconds)
    {
        motor1.setPower(MotorSpeed);
        motor2.setPower(-MotorSpeed);
        motor3.setPower(MotorSpeed);
        motor4.setPower(-MotorSpeed);
        sleep(miliseconds);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    private void UpdateVoltage()
    {
        //Voltaju cu care vrei sa mearga motoarele
        VoltajDorit = 11;

        //Voltajul actual
        VoltajActual = VoltageSensor0.getVoltage();

        //raportul pentru calcularea a vitezei
        Raport = VoltajDorit/VoltajActual;

        //Viteza motoarelor modificata de raport
        MotorSpeed = 0.5 * Raport;
    }
    private void UpdatePid()
    {
        controller = new PIDController(Kp, Ki, Kd);
        controller.setPID(Kp, Ki, Kd);
        int armpos = motorbrat.getCurrentPosition();
        double Kpid = controller.calculate(armpos,target);
        double Kff = Math.cos(Math.toRadians(target / ticks_in_degree)) * Kf;
        double putere_pid = Kpid + Kff;
        motorbrat.setPower(putere_pid);
    }
    private void DoNothing(long mili)
    {
        sleep(mili);
    }

}