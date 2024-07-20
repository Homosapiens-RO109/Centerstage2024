package org.firstinspires.ftc.teamcode.arghh;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DDrive {
    public static double x;
    public static double y;
    public static double rt;
    public static double den;
    public static double powFD;
    public static double powFS;
    public static double powSS;
    public static double powSD;
    final DcMotor motorFD;
    final DcMotor motorFS;
    final DcMotor motorSS;
    final DcMotor motorSD;
    HardwareMap hardwareMap;
    Gamepad gamepad;
    public DDrive(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
        this.motorFS = hardwareMap.get(DcMotor.class, "mFS");
        this.motorFD = hardwareMap.get(DcMotor.class, "mFD");
        this.motorSD = hardwareMap.get(DcMotor.class, "mSD");
        this.motorSS = hardwareMap.get(DcMotor.class, "mSS");
        motorSD.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFD.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void movement() {
        x = this.gamepad.left_stick_x;
        y = -this.gamepad.left_stick_y;
        rt = this.gamepad.right_trigger - this.gamepad.left_trigger;
        den = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rt), 1);

        powFS = (y + x + rt) / den;
        powFD = (y - x - rt) / den;
        powSS = (y - x + rt) / den;
        powSD = (y + x - rt) / den;

        motorFS.setPower(powFS * 0.8);
        motorFD.setPower(powFD * 0.8);
        motorSS.setPower(powSS * 0.8);
        motorSD.setPower(powSD * 0.8);
    }
}