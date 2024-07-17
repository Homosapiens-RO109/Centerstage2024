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

public class inteik{

    private DcMotor motorIn;
    private CRServo servoIn;
    public Servo servoDrop;
    Gamepad gamepad1;
    HardwareMap hardwareMap;
    ElapsedTime timerIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public inteik(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }

    void hwmp() {
        motorIn = hardwareMap.get(DcMotor.class, "motorIN");
        servoIn = hardwareMap.get(CRServo.class,"servoIN");
        servoDrop = hardwareMap.get(Servo.class,"servoDrop");

        servoDrop.setPosition(0);
    }

    void StartIn() {
        servoDrop.setPosition(0.7);
        motorIn.setPower(1);
        servoIn.setPower(-1);
        timerIntake.reset();
    }

    void StopIn() {
        servoDrop.setPosition(0);
        motorIn.setPower(0);
        servoIn.setPower(0);
        timerIntake.reset();
    }

    public void ControlIn() {
        if(gamepad1.a)
        {
            if(motorIn.getPower() == 0 && timerIntake.time(TimeUnit.MILLISECONDS) >= 500)
                StartIn();
            if(motorIn.getPower() != 0 && timerIntake.time(TimeUnit.MILLISECONDS) >= 500)
                StopIn();
        }
    }
}
