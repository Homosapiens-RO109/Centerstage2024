package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class robot_intake {
    FtcDashboard dashboard;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public DcMotor motorIntake;
    public Servo servoIntake;
    public int timerIntake;
    public robot_intake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    public void hwmp() {
        motorIntake = hardwareMap.get(DcMotor.class, "motorin");
        servoIntake = hardwareMap.get(Servo.class, "servoin");
        dashboard = FtcDashboard.getInstance();
    }

    public void power() {
        if(motorIntake.getPower() == 0) {
            if(timerIntake > 15) {
                if (gamepad1.left_bumper) {
                    motorIntake.setPower(0.5);
                    timerIntake = 0;
                }

                if (gamepad1.right_bumper) {
                    motorIntake.setPower(-0.5);
                    timerIntake = 0;
                }
            }
        }

        if(motorIntake.getPower() != 0)
            if(timerIntake > 15 && (gamepad1.right_bumper || gamepad1.left_bumper)) {
                motorIntake.setPower(0);
                timerIntake = 0;
            }

        timerIntake++;

        if(timerIntake > 2000)
            timerIntake = 50;
    }
}