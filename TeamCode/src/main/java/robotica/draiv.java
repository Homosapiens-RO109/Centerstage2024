package robotica;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.concurrent.TimeUnit;
public class draiv {


    public double x,y, rt, den, powFD, powFS, powSS, powSD;
    public DcMotor motorFD, motorFS, motorSS, motorSD;
    HardwareMap hardwareMap;
    Gamepad gamepad1;

    void hwmp() {
        motorFD = hardwareMap.get(DcMotor.class, "motorFD");
        motorFS = hardwareMap.get(DcMotor.class, "motorFS");
        motorSS = hardwareMap.get(DcMotor.class, "motorSS");
        motorSD = hardwareMap.get(DcMotor.class, "motorSD");
        motorSD.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFD.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public draiv(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    public void movement() {
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rt = gamepad1.right_trigger - gamepad1.left_trigger;
        den = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rt), 1);

        powFS = (y + x + rt) / den;
        powFD = (y - x - rt) / den;
        powSS = (y - x + rt) / den;
        powSD = (y + x - rt) / den;

        motorFS.setPower(powFS);
        motorFD.setPower(powFD);
        motorSS.setPower(powSS);
        motorSD.setPower(powSD);
    }
}
