package robotica;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class outeik {
    public Servo servoSt, servoDr, servoWrist, servoDegetica;
    public DcMotor motorst, motordr;
    public static double error = 0, servopoz = 0, kp = 0.0001, ki, kd, kfst, kfdr;

    public static int target;

    public final double ticks_in_degree = 532/360.0;
    Gamepad gamepad1;
    HardwareMap hardwareMap;
    FtcDashboard dashboard;
    PIDController pid =  new PIDController(kp, ki, kd);

    public outeik(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        hwmp();
    }
    void hwmp() {
        motorst = hardwareMap.get(DcMotor.class, "glisst");
        motordr = hardwareMap.get(DcMotor.class, "glisdr");
        motordr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorst.setDirection(DcMotorSimple.Direction.REVERSE);
        target = 0;

        servoSt = hardwareMap.get(Servo.class,"servost");
        servoDr = hardwareMap.get(Servo.class, "servodr");
        servoWrist = hardwareMap.get(Servo.class, "servoWrist");
        servoDegetica = hardwareMap.get(Servo.class, "servoDegetica");

        servoDr.setDirection(Servo.Direction.REVERSE);
        servoWrist.setDirection(Servo.Direction.REVERSE);
        servoSt.setPosition(0.2);
        servoDr.setPosition(0.2);
        servoWrist.setPosition(0.1);
        pid = new PIDController(kp, ki, kd);
        dashboard = FtcDashboard.getInstance();
    }

    boolean SlidersExtended() {
        return target > 1000;
    }

    void pozitionareBrat1() {
        servoSt.setPosition(0.2);
        servoDr.setPosition(0.2+error);
        servoWrist.setPosition(0.1);
    }

    void pozitionareBrat2() {
        servoSt.setPosition(0.9);
        servoDr.setPosition(0.9+error);
        servoWrist.setPosition(0.5);
    }

    public void ControlBrat() {
        if(!SlidersExtended())
            pozitionareBrat1();
        if(SlidersExtended())
            pozitionareBrat2();
    }

    void ForceStop(double multiplier) {
        if(target > 3000)
            target = 3000;
        if(target < 0)
            target = 0;
        if(multiplier == 0)
            target = motorst.getCurrentPosition();

    }
    public void miscaSlider(double multiplier) {
        target += multiplier;
        ForceStop(multiplier);
    }
    public void miscaPID() {
        pid.setPID(kp, ki, kd);
        double powerLeftSlider = pid.calculate(motorst.getCurrentPosition(), target);
        double powerRightSlider = pid.calculate(motordr.getCurrentPosition(), target);
        double kfLeftSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
        double kfRighSlider = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
        motorst.setPower(powerLeftSlider + kfLeftSlider);
        motordr.setPower(powerRightSlider + kfRighSlider);
    }

}
