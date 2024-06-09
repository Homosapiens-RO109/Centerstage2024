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
    public DcMotor motorLeftSlider, motorRightSlider;
    public Servo servoLeft, servoRight, servoBox, servoWrist;

    public static double kp = 0.001, ki, kd, kfst, kfdr, ticks_in_degree = 532/360.0;
    public static int target;
    final int MAX_RANGE_SLIDERS = 2800, MIN_RANGE_SLIDERS = 0;
    final double POS_FINAL_BRAT = 0.7, POS_INITIAL_BRAT = 0.2, POS_INITIAL_WRIST = 0, POS_FINAL_WRIST = 0.66;

    public robot_outtake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    public void hwmp() {
        motorLeftSlider = hardwareMap.get(DcMotor.class, "Sliderst");
        motorRightSlider = hardwareMap.get(DcMotor.class, "Sliderdr");
        motorRightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        servoLeft = hardwareMap.get(Servo.class, "servost");
        servoRight = hardwareMap.get(Servo.class, "servodr");
        servoBox = hardwareMap.get(Servo.class, "servorelease");
        servoWrist = hardwareMap.get(Servo.class, "servowrist");
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoLeft.setPosition(0);
        servoRight.setPosition(0);

        pid = new PIDController(kp, ki, kd);
        dashboard = FtcDashboard.getInstance();
    }

    boolean SlidersExtended() {
        return target > 1000;
    }
    boolean SlidersWithinRange(double target) {
        return MIN_RANGE_SLIDERS < target && target < MAX_RANGE_SLIDERS;
    }
    
    boolean SlidersOutOfRange(double target) {
        return target < MIN_RANGE_SLIDERS || target > MAX_RANGE_SLIDERS;
    }
    
    boolean InputInRange(double target, double r_stick_y) {
        return target + r_stick_y > MIN_RANGE_SLIDERS && target + r_stick_y < MAX_RANGE_SLIDERS;
    }
    public void ExtendSliders(double multiplier)
    {
        if(SlidersWithinRange(target) || SlidersOutOfRange(target) && InputInRange(target, multiplier))
            target += multiplier;
    }
    
    void UseServos() {
        if(SlidersExtended()) {
            servoWrist.setPosition(POS_FINAL_WRIST);
            servoLeft.setPosition(POS_FINAL_BRAT);
            servoRight.setPosition(POS_FINAL_BRAT);
        }

        if(!SlidersExtended()) {
            servoWrist.setPosition(POS_INITIAL_WRIST);
            servoLeft.setPosition(POS_INITIAL_BRAT);
            servoRight.setPosition(POS_INITIAL_BRAT);
        }
    }
    public void SlidersPID() {
        pid.setPID(kp, ki, kd);
        double powerLeftSlider = pid.calculate(motorLeftSlider.getCurrentPosition(), target);
        double powerRightSlider = pid.calculate(motorRightSlider.getCurrentPosition(), target);
        double kfLeftSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
        double kfRighSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
        motorLeftSlider.setPower(powerLeftSlider + kfLeftSlider);
        motorRightSlider.setPower(powerRightSlider + kfRighSlider);
    }
}
