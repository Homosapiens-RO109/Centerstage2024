package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class robot_intake {
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public DcMotor motorIntake;
    public Servo servoIntake;
    public int timerIntake;
    final int MAX_TIMER_REACH = 2000, TIMER_RESET = 50;
    public robot_intake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    public void hwmp() {
        motorIntake = hardwareMap.get(DcMotor.class, "motorin");
        servoIntake = hardwareMap.get(Servo.class, "servoin");
    }
    public void TimerUntilReach() {
        timerIntake++;
        if(timerIntake >= MAX_TIMER_REACH)
            timerIntake = TIMER_RESET;
    }
    void StopIntake() {
        motorIntake.setPower(0);
        timerIntake = 0;
    }

    void StartIntake(double power) {
        motorIntake.setPower(power);
        timerIntake = 0;
    }
    public void ControlIntake() {
        if(timerIntake > 15) {
            if (motorIntake.getPower() == 0) {
                if (gamepad1.left_bumper)
                    StartIntake(0.5);

                if (gamepad1.right_bumper)
                    StartIntake(-0.5);
            }

            if(motorIntake.getPower() != 0 && (gamepad1.left_bumper || gamepad1.right_bumper))
                StopIntake();
        }
        TimerUntilReach();
    }
}