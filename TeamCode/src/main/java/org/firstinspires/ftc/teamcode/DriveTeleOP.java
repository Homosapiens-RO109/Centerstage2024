package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.videoio.VideoCapture;
@Disabled
@Config
@TeleOp


public class DriveTeleOP extends LinearOpMode {
    private Servo servo;
    boolean IsOpen = false;

    private PIDController controller;
    public static double Kp = 0.00157,Ki = 0.00001, Kd = 0.0001, Kf = 0.001;
    private static VideoCapture cam = new VideoCapture(0);
//    private static Scalar lower_blue = new Scalar(), upper_blue = new Scalar(), lower_red = new Scalar(), upper_red = new Scalar();

    //Kp = 0.0017,Ki = 0, Kd = 0.00025, Kf = 0.001;
    public static int target = 0;

    private final double ticks_in_degree = 312 / 180.0;

    public FtcDashboard dashboard;
    private DcMotorEx motorbrat;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("stangasus");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("stangajos");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("d" +
                "" +
                "reaptasus");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("dreaptajos");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servo.setPosition(0.9);

        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(Kp, Ki, Kd);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        motorbrat = hardwareMap.get(DcMotorEx.class, "motorbrat");
        motorbrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // esti prajit

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Movement input
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

            //Movement
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -((y - x + rx) / denominator);
            double backLeftPower = -((y + x + rx) / denominator);
            double frontRightPower = -((y + x - rx) / denominator);
            double backRightPower = -((y - x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower * 0.5);
            backLeftMotor.setPower(backLeftPower * 0.5);
            frontRightMotor.setPower(frontRightPower * 0.62);
            backRightMotor.setPower(backRightPower * 0.5);

//            if(cam.isOpened())
//            {
//                Mat frame = new Mat();
//                cam.read(frame);
//            }


            //Deschidere si inchidere claw

            //PID controller calcule
            controller.setPID(Kp, Ki, Kd);
            int armpos = motorbrat.getCurrentPosition();
            double Kpid = controller.calculate(armpos, target);
            double Kff = Math.cos(Math.toRadians(target / ticks_in_degree)) * Kf;

            double putere_pid = Kpid + Kff;

            motorbrat.setPower(putere_pid);
            telemetry.addData("pos", armpos);
            telemetry.addData("target", target);

//            if(armpos < 0)
//            {
//                Kf = -0.01;
//            }
//            else Kf = 0.01;

            if (IsOpen) {
                //Pozitie 90 grade
                if (gamepad1.dpad_down) {
                    servo.setPosition(0.9);
                    sleep(200);
                    target = -69;
                }

                //Pozitie put pixel
                if (gamepad1.left_bumper) {
                    servo.setPosition(0.9);
                    sleep(200);
                    target = 120;

                }
                if (gamepad1.right_bumper) {
                    servo.setPosition(0.9);
                    sleep(200);
                    target = -250;
                }
                //Pozitie grab pixel
                if (gamepad1.b) {
                    servo.setPosition(0.9);
                    sleep(200);
                    target = -345;

                }
                //Pozitie defensiva
                if (gamepad1.y) {
                    servo.setPosition(0.9);
                    sleep(200);
                    target = 300;

                }
            } else {
                if (gamepad1.dpad_down) {
                    target = -69;
                }
                if (gamepad1.left_bumper) {
                    target = 120;
                }
                //Pozitie put pixel
                if (gamepad1.b) {
                    target = -345;
                }
                if (gamepad1.right_bumper) {
                    target = -250;
                }
                //Pozitie defensiva
                if (gamepad1.y) {
                    target = 300;
                }

            }
            if (IsOpen) {
                if (gamepad1.a) {
                    IsOpen = false;
                    servo.setPosition(0.9);
                    sleep(200);
                }
            }
            else {
                if (gamepad1.a) {
                    IsOpen = true;
                    servo.setPosition(0.7);
                    sleep(200);
                }
            }
            double asde = servo.getPosition();
            telemetry.addData("pozitie", asde);
            telemetry.addData("bol", IsOpen);
            telemetry.update();
        }


    }
}

