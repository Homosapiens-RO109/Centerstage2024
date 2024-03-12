package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@Config
@TeleOp
public class DriveFieldBased extends LinearOpMode {
    private PIDController controller;
    public static double Kp = 0.001,Ki = 0, Kd = 0;
    public FtcDashboard dashboard;
    private DcMotor MotorFL,MotorFR,MotorRL,MotorRR;
    private DcMotorEx MotorGL,MotorGR;
    private final double ticks_in_degree = 312 / 180.0;
    public static int target = 0;
    public static int Gvelo = 200;
    @Override
    public void runOpMode() throws InterruptedException{
        MotorFL = hardwareMap.get(DcMotor.class, "stsus");
        MotorFR = hardwareMap.get(DcMotor.class, "drsus");
        MotorRL = hardwareMap.get(DcMotor.class, "stjos");
        MotorRR = hardwareMap.get(DcMotor.class, "drjos");
        MotorGL = hardwareMap.get(DcMotorEx.class, "glis1");
        MotorGR = hardwareMap.get(DcMotorEx.class, "glis2");
        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRL.setDirection(DcMotorSimple.Direction.REVERSE);


        MotorGL.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(Kp, Ki, Kd);
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MotorGL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorGR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        waitForStart();
        if(isStopRequested())
            return;
        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

            if(gamepad1.start)
                imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double PutereFL = (rotY + rotX + rx) / denominator;
            double PutereRL = (rotY - rotX + rx) / denominator;
            double PutereFR = (rotY - rotX - rx) / denominator;
            double PutereRR = (rotY + rotX - rx) / denominator;

            MotorFL.setPower(PutereFL);
            MotorRL.setPower(PutereRL);
            MotorFR.setPower(PutereFR);
            MotorRR.setPower(PutereRR);

            controller.setPID(Kp, Ki, Kd);
            int glisposl = MotorGL.getCurrentPosition();
            int glisposr = MotorGR.getCurrentPosition();
            double KPIDL = controller.calculate(glisposl, target);
            double KPIDR = controller.calculate(glisposr, target);

            double pow_pidl = KPIDL;
            double pow_pidr = KPIDR;

            MotorGL.setPower(pow_pidl);
            MotorGR.setPower(pow_pidr);

            if(gamepad1.dpad_up)
                target = 3400;
            if(gamepad1.dpad_down)
                target = 0;
            if(gamepad1.dpad_left)
                target = 1085;
            if(gamepad1.dpad_right)
                target = 2150;

            telemetry.addData("Pozitie glisanta dreapta", MotorGR.getCurrentPosition());
            telemetry.addData("Pozitie glisanta stanga", MotorGL.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }
}
