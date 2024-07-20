package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class robot_drive {
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public DcMotor motorLeftFront, motorLeftBack, motorRightFront, motorRightBack;
    public double x, y, powFL, powFR, powRL, powRR, den, rt;

    public robot_drive(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        hwmp();
    }

    public void hwmp() {
        motorLeftFront = hardwareMap.get(DcMotor.class, "stsus");
        motorRightFront = hardwareMap.get(DcMotor.class, "drsus");
        motorLeftBack = hardwareMap.get(DcMotor.class, "stjos");
        motorRightBack = hardwareMap.get(DcMotor.class, "drjos");
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void movement() {
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rt = gamepad1.right_trigger - gamepad1.left_trigger;
        den = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rt), 1);

        powFL = (y + x + rt) / den;
        powFR = (y - x - rt) / den;
        powRL = (y - x + rt) / den;
        powRR = (y + x - rt) / den;

        motorLeftFront.setPower(powFL);
        motorRightFront.setPower(powFR);
        motorLeftBack.setPower(powRL);
        motorRightBack.setPower(powRR);
    }
}