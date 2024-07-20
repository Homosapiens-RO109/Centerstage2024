package org.firstinspires.ftc.teamcode.Autonomie;//package org.firstinspires.ftc.teamcode.Autonomie;
//
//
//import com.arcrobotics.ftclib.controller.PIDController;
//
//public class Autonom extends Hardwaremap {
//
//    public void set_target(int target)
//    {
//        Autonom.target = target;
//    }
//
//    public void stop_motors()
//    {
//        motor1.setPower(0);
//        motor2.setPower(0);
//        motor3.setPower(0);
//        motor4.setPower(0);
//    }
//
//    public void DriveBackwards(long miliseconds)
//    {
//        motor1.setPower(-MotorSpeed);
//        motor2.setPower(-MotorSpeed);
//        motor3.setPower(-MotorSpeed);
//        motor4.setPower(-MotorSpeed);
//        sleep(miliseconds);
//        stop_motors();
//    }
//
//    public void DriveFront(long miliseconds)
//    {
//        motor1.setPower(MotorSpeed);
//        motor2.setPower(MotorSpeed);
//        motor3.setPower(MotorSpeed);
//        motor4.setPower(MotorSpeed);
//        sleep(miliseconds);
//        stop_motors();
//    }
//
//    public void DriveLeft(long miliseconds)
//    {
//        motor1.setPower(-MotorSpeed);
//        motor2.setPower(MotorSpeed);
//        motor3.setPower(-MotorSpeed);
//        motor4.setPower(MotorSpeed);
//        sleep(miliseconds);
//        stop_motors();
//    }
//    public void DriveRight(long miliseconds)
//    {
//        motor1.setPower(MotorSpeed);
//        motor2.setPower(-MotorSpeed);
//        motor3.setPower(MotorSpeed);
//        motor4.setPower(-MotorSpeed);
//        sleep(miliseconds);
//        stop_motors();
//    }
//    public void UpdateVoltage()
//    {
//        //Voltaju cu care vrei sa mearga motoarele
//        VoltajDorit = 11;
//
//        //Voltajul actual
//        VoltajActual = VoltageSensor0.getVoltage();
//
//        //raportul pentru calcularea a vitezei
//        Raport = VoltajDorit / VoltajActual;
//
//        //Viteza motoarelor modificata de raport
//        MotorSpeed = 0.5 * Raport;
//    }
//    public void UpdatePid()
//    {
//        controller = new PIDController(Kp, Ki, Kd);
//        controller.setPID(Kp, Ki, Kd);
//        int armpos = motorbrat.getCurrentPosition();
//        double Kpid = controller.calculate(armpos,target);
//        double Kff = Math.cos(Math.toRadians(target / ticks_in_degree)) * Kf;
//        double putere_pid = Kpid + Kff;
//        motorbrat.setPower(putere_pid);
//    }
//}
