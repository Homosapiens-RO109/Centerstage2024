package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class robot_outtake {
    FtcDashboard dashboard;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public PIDController pid;
    public DcMotor motorLeftGlis, motorRightGlis;
    public Servo servoLeft, servoRight, servoBox, servoWrist;

    public static double kp = 0.001, ki, kd, kfst, kfdr, ticks_in_degree = 532/360.0, p00la = 0.08, extend_servo = 0.9;
    public static int target;

    public robot_outtake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    public void hwmp() {
        motorLeftGlis = hardwareMap.get(DcMotor.class, "glisst");
        motorRightGlis = hardwareMap.get(DcMotor.class, "glisdr");
        motorRightGlis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightGlis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftGlis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftGlis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightGlis.setDirection(DcMotorSimple.Direction.REVERSE);

        servoLeft = hardwareMap.get(Servo.class, "servost");
        servoRight = hardwareMap.get(Servo.class, "servodr");
        servoBox = hardwareMap.get(Servo.class, "servorelease");
        servoWrist = hardwareMap.get(Servo.class, "servowrist");
        servoLeft.setPosition(p00la);
        servoRight.setPosition(p00la);

        pid = new PIDController(kp, ki, kd);
        dashboard = FtcDashboard.getInstance();
    }
    
    public void Glis_pid() {
        pid.setPID(kp, ki, kd);
        double powerLeftGlis = pid.calculate(motorLeftGlis.getCurrentPosition(), target);
        double powerRightGlis = pid.calculate(motorRightGlis.getCurrentPosition(), target);
        double kfLeftGlis = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
        double kfRighGlis = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
        motorLeftGlis.setPower(powerLeftGlis + kfLeftGlis);
        motorRightGlis.setPower(powerRightGlis + kfRighGlis);
        target += 50 * gamepad1.right_stick_y;
    }
}
