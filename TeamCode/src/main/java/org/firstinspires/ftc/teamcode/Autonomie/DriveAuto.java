package org.firstinspires.ftc.teamcode.Autonomie;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveAuto {
    public DcMotor motorFL, motorRL, motorFR, motorRR, motorRoata, motorGL, motorGR;
    public Servo servoBS, servoBD, servowr;
    public static double Servobsjos = 0.07,servobssus=0.58,Servobdjos=0.11,Servobdsus=0.62,Servowrinchis=0.85,Servowrdeschis=0.4;
    boolean inchis = true;
    final int ticks_motorFL = 536, ticks_motorFR = 529, ticks_motorRL = 538, ticks_motorRR = 525;
    PID pid = new PID();
    public void GlisPID(int target)
    {
        pid.target = target;
        pid.calculate();

        if(motorGL.getPower() < 0)
            pid.kp = 0.001;
        else 
            pid.kp = 0.004;
        
        motorGL.setPower(pid.KPIDL);
        motorGR.setPower(pid.KPIDR);

        if(motorGL.getCurrentPosition() <= 1200 && pid.target == 0 && motorGL.getPower() < 0){
            servoBS.setPosition(Servobsjos);
            servoBD.setPosition(Servobdjos);
        }
        if(motorGL.getCurrentPosition() >= 1050 && pid.target >= 1080) {
            servoBS.setPosition(servobssus);
            servoBD.setPosition(Servobdsus);
        }
        if(motorGL.getCurrentPosition() <= 500 && !inchis)
        {
            servowr.setPosition(Servowrinchis);
            inchis = true;
        }
    }

    public void stop_motors()
    {
        motorRL.setPower(0);
        motorRR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void DriveForward()
    {
        motorFL.setTargetPosition(ticks_motorFL + motorFL.getCurrentPosition());
        motorRL.setTargetPosition(ticks_motorRL + motorRL.getCurrentPosition());
        motorRR.setTargetPosition(ticks_motorRR + motorRR.getCurrentPosition());
        motorFL.setTargetPosition(ticks_motorFL + motorFL.getCurrentPosition());
        motorFR.setTargetPosition(ticks_motorFR + motorFR.getCurrentPosition());

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(0.5);
        motorRR.setPower(0.5);
        motorFL.setPower(0.5);
        motorFR.setPower(0.5);
    }

    public void DriveBackwards()
    {
        motorRL.setTargetPosition(-ticks_motorRL + motorRL.getCurrentPosition());
        motorRR.setTargetPosition(-ticks_motorRR + motorRR.getCurrentPosition());
        motorFL.setTargetPosition(-ticks_motorFL + motorFL.getCurrentPosition());
        motorFR.setTargetPosition(-ticks_motorFR + motorFR.getCurrentPosition());

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-0.5);
        motorRR.setPower(-0.5);
        motorFL.setPower(-0.5);
        motorFR.setPower(-0.5);
    }

    public void DriveLeft()
    {
        motorRL.setTargetPosition(ticks_motorRL + motorRL.getCurrentPosition());
        motorRR.setTargetPosition(-ticks_motorRR + motorRR.getCurrentPosition());
        motorFL.setTargetPosition(-ticks_motorFL + motorFL.getCurrentPosition());
        motorFR.setTargetPosition(ticks_motorFR + motorFR.getCurrentPosition());

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(0.5);
        motorRR.setPower(-0.5);
        motorFL.setPower(-0.5);
        motorFR.setPower(0.5);
    }
    public void DriveRight()
    {
            motorRL.setTargetPosition(-ticks_motorRL + motorRL.getCurrentPosition());
            motorRR.setTargetPosition(ticks_motorRR + motorRR.getCurrentPosition());
            motorFL.setTargetPosition(ticks_motorFL + motorFL.getCurrentPosition());
            motorFR.setTargetPosition(-ticks_motorFR + motorFR.getCurrentPosition());

            motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorRL.setPower(-0.5);
            motorRR.setPower(0.5);
            motorFL.setPower(0.5);
            motorFR.setPower(-0.5);
    }

    public void RotateLeft()
    {
        motorRL.setTargetPosition(-ticks_motorRL + motorRL.getCurrentPosition());
        motorRR.setTargetPosition(ticks_motorRR + motorRR.getCurrentPosition());
        motorFL.setTargetPosition(-ticks_motorFL + motorFL.getCurrentPosition());
        motorFR.setTargetPosition(ticks_motorFR + motorFR.getCurrentPosition());

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-1);
        motorRR.setPower(1);
        motorFL.setPower(-1);
        motorFR.setPower(1);
    }

    public void RotateRight()
    {
        motorRL.setTargetPosition(ticks_motorRL + motorRL.getCurrentPosition());
        motorRR.setTargetPosition(-ticks_motorRR + motorRR.getCurrentPosition());
        motorFL.setTargetPosition(ticks_motorFL + motorFL.getCurrentPosition());
        motorFR.setTargetPosition(-ticks_motorFR + motorFR.getCurrentPosition());

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(1);
        motorRR.setPower(-1);
        motorFL.setPower(1);
        motorFR.setPower(-1);
    }
}