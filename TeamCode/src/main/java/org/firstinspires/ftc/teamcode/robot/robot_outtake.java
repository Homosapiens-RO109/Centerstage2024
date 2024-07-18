package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.function.Min;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class robot_outtake {
    FtcDashboard dashboard;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public PIDController pid;
    public DcMotor MotorLeftSlider, MotorRightSlider;
    public Servo ServoLeft, ServoRight, ServoBox, ServoWrist;

    public static double kp = 0.001, ki, kd, kfst, kfdr, ticks_in_degree = 532/360.0;
    public static int target;
    final int MAX_RANGE_SLIDERS = 2800, MIN_RANGE_SLIDERS = 0, EXTEND_RANGE = 1000;
    final double POS_FINAL_ARM = 0.7, POS_INITIAL_ARM = 0.2, POS_INITIAL_WRIST = 0, POS_FINAL_WRIST = 0.66;

    public robot_outtake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    public void hwmp() {
        MotorLeftSlider = hardwareMap.get(DcMotor.class, "glisst");
        MotorRightSlider = hardwareMap.get(DcMotor.class, "glisdr");
        MotorRightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorLeftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorRightSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        
        ServoLeft = hardwareMap.get(Servo.class, "servost");
        ServoRight = hardwareMap.get(Servo.class, "servodr");
        ServoBox = hardwareMap.get(Servo.class, "servobox");
        ServoWrist = hardwareMap.get(Servo.class, "servowrist");
        target = 0;
        ServoRight.setDirection(Servo.Direction.REVERSE);
        ServoRight.setPosition(0);
        ServoLeft.setPosition(0);
        pid = new PIDController(kp, ki, kd);
        dashboard = FtcDashboard.getInstance();
    }
    boolean SlidersExtended() {
        return target > EXTEND_RANGE;
    }

    public void TargetForceStop(double multiplier) {
        if(target > MAX_RANGE_SLIDERS)
            target = MAX_RANGE_SLIDERS;

        if(target < MIN_RANGE_SLIDERS)
            target = MIN_RANGE_SLIDERS;

        if(multiplier == 0)
            target = MotorLeftSlider.getCurrentPosition();
    }
    public void MoveSliders(double multiplier) {
        target += multiplier;
        TargetForceStop(multiplier);
    }
    
    public void UseServos() {
        if(!SlidersExtended()) {
            ServoWrist.setPosition(POS_INITIAL_WRIST);
            ServoLeft.setPosition(POS_INITIAL_ARM);
            ServoRight.setPosition(POS_INITIAL_ARM);
        }
        
        if(SlidersExtended()) {
            ServoWrist.setPosition(POS_FINAL_WRIST);
            ServoLeft.setPosition(POS_FINAL_ARM);
            ServoRight.setPosition(POS_FINAL_ARM);
        }
    }
    public void SlidersPID() {
        pid.setPID(kp, ki, kd);
        double powerLeftSlider = pid.calculate(MotorLeftSlider.getCurrentPosition(), target);
        double powerRightSlider = pid.calculate(MotorRightSlider.getCurrentPosition(), target);
        double kfLeftSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
        double kfRighSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
        MotorLeftSlider.setPower(powerLeftSlider + kfLeftSlider);
        MotorRightSlider.setPower(powerRightSlider + kfRighSlider);
    }
}