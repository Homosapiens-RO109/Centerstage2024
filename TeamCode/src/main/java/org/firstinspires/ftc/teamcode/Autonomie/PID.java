package org.firstinspires.ftc.teamcode.Autonomie;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PID {
    PIDController controller;

    private DcMotor MotorFL,MotorFR,MotorRL,MotorRR, MotorGL, MotorGR, MotorRoata;

    public static double kd, ki , kp = 0.004;
    public static int target = 0;
    public double KPIDL, KPIDR;
    public void ref(Hardware hard)
    {
        MotorGL = hard.motorGL;
        MotorGR = hard.motorGR;
        MotorGL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void calculate()
    {
        controller = new PIDController(kp, ki, kd);
        KPIDL = controller.calculate(MotorGL.getCurrentPosition(), target);
        KPIDR = controller.calculate(MotorGR.getCurrentPosition(), target);
    }
}
