package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
public class robot_intake {
    ElapsedTime timerIntake = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public DcMotor motorIntake;
    public Servo servoIntake;
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
    void StopIntake() {
        motorIntake.setPower(0);
        timerIntake.reset();
    }

    void StartIntake(double power) {
        motorIntake.setPower(power);
        timerIntake.reset();
    }
    public void ControlIntake() {
        if(timerIntake.now(TimeUnit.SECONDS) > 1) {
            if (motorIntake.getPower() == 0) {
                if (gamepad1.left_bumper)
                    StartIntake(0.5);

                if (gamepad1.right_bumper)
                    StartIntake(-0.5);
            }

            if(motorIntake.getPower() != 0 && (gamepad1.left_bumper || gamepad1.right_bumper))
                StopIntake();
        }
    }
}