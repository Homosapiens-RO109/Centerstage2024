//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@Config
//@TeleOp
//public class PIDF extends OpMode {
//    private PIDController controller;
//    public static double p = 0,i = 0, d = 0, f = 0.15;
//    public static int target = 0;
//
//    private final double ticks_in_degree = 312 / 180.0;
//
//    private DcMotorEx motorbrat;
//
//    @Override
//    public void init()
//    {
//        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        motorbrat = hardwareMap.get(DcMotorEx.class, "motorbrat");
//    }
//    @Override
//    public double loop()
//    {
//        controller.setPID(p, i, d);
//        int armpos = motorbrat.getCurrentPosition();
//        if(armpos >= -400) {
//            f = -0.17;
//        }
//        else {
//            f = 0.17;
//        }
//        double pid = controller.calculate(armpos,target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//
//        double power_pid = pid + ff;
//
//        motorbrat.setPower(power_pid);
//        telemetry.addData("pos", armpos);
//        telemetry.addData("target", target);
//        telemetry.update();
//
//        return power_pid;
//    }
//}
