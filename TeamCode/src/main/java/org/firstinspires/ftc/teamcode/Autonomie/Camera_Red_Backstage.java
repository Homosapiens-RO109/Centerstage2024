package org.firstinspires.ftc.teamcode.Autonomie;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Red_Detection_Backstage")
public class Camera_Red_Backstage extends LinearOpMode {

    DcMotor motorFL, motorFR, motorRL, motorRR, motorGL, motorGR, motorRoata;
    Servo servoBS, servoBD, servowr;
    public static double kp,ki,kd;
    public static int target = 0;
    PIDController controller;

    boolean inchis = true;
    final int ticks_motorFL = 536, ticks_motorFR = 529, ticks_motorRL = 538, ticks_motorRR = 525;
    public static double Servobsjos = 0.04,Servobssus=0.58,Servobdjos=0.08,Servobdsus=0.62,Servowrinchis=0.85,Servowrdeschis=0.4;
    int pas = 0;
    boolean detect = true;
    boolean ED = false;
    Pipeline_Red detection = new Pipeline_Red();
    OpenCvWebcam webcam;
    Pipeline_Red.SkystonePosition pozitie;

    @Override
    public void runOpMode() throws InterruptedException{

        motorFL = hardwareMap.get(DcMotor.class, "stsus");
        motorFR = hardwareMap.get(DcMotor.class, "drsus");
        motorRL = hardwareMap.get(DcMotor.class, "stjos");
        motorRR = hardwareMap.get(DcMotor.class, "drjos");
        motorGL = hardwareMap.get(DcMotor.class, "glis1");
        motorGR = hardwareMap.get(DcMotor.class, "glis2");
        motorRoata = hardwareMap.get(DcMotor.class, "m_roata");

        servoBD = hardwareMap.get(Servo.class, "Servobd");
        servoBS = hardwareMap.get(Servo.class, "Servobs");
        servowr = hardwareMap.get(Servo.class, "Servowr");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRoata.setDirection(DcMotor.Direction.REVERSE);
        motorGL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorGL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorGR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = 0;


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int webcamID = hardwareMap.appContext.getResources().getIdentifier("webcamID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, webcamID);
        webcam.setPipeline(detection);
        controller = new PIDController(kp,ki,kd);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while(!isStarted())
        {
            pozitie = detection.getAnalysis();
            telemetry.addData("Position: ", pozitie);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive())
        {
            controller.setPID(kp, ki, kd);
            double KPIDL = controller.calculate(motorGL.getCurrentPosition(), target);
            double KPIDR = controller.calculate(motorGR.getCurrentPosition(), target);

            if(motorGL.getPower() < 0)
                kp = 0.001;
            else kp = 0.004;
            motorGL.setPower(KPIDL);
            motorGR.setPower(KPIDR);

            if(motorGL.getCurrentPosition() <= 1200 && target == 0 && motorGL.getPower() < 0){
                servoBS.setPosition(Servobsjos);
                servoBD.setPosition(Servobdjos);
            }
            if(motorGL.getCurrentPosition() >= 1050 && target >= 1080) {
                servoBS.setPosition(Servobssus);
                servoBD.setPosition(Servobdsus);
            }
            if(motorGL.getCurrentPosition() <= 500 && !inchis)
            {
                servowr.setPosition(Servowrinchis);
                inchis = true;
            }

            if(pozitie == Pipeline_Red.SkystonePosition.LEFT)
            {
                if(pas == 0){
                    DriveBackwards(2.3);
                    pas++;
                }

                if(pas == 1 && !motorFL.isBusy())
                {
                    pas++;
                    DriveRight(0.4);
                }

                if(pas == 2 && !motorFL.isBusy())
                {
                    RotateRight(2);
                    pas++;
                }

                if(pas == 3 && !motorFL.isBusy())
                {
                    DriveBackwards(0.2);
                    pas++;
                }

                if(pas == 4 && !motorFL.isBusy()) {
                    motorRoata.setPower(-1);
                    sleep(500);
                    motorRoata.setPower(0);
                    pas++;
                }
            }

            if(pozitie == Pipeline_Red.SkystonePosition.CENTER)
            {
                if(pas == 0){
                    DriveBackwards(2.5);
                    sleep(1800);
                    DriveForward(0.45);
                    pas++;
                }
                if(pas == 1 && !motorFL.isBusy()) {
                    pas++;
                    motorRoata.setPower(-1);
                    sleep(500);
                    motorRoata.setPower(0);
                    DriveForward(0.2);
                }
                if(pas == 2 && !motorFL.isBusy())
                {
                    RotateRight(1.5);
                    sleep(300);
                    pas++;
                }
                if(pas == 3 && !motorFL.isBusy())
                {
                    target = 1080;
                    DriveForward(3);
                    pas++;
                }
                if(pas == 4 && !motorFL.isBusy())
                {
                    servowr.setPosition(Servowrdeschis);
                    sleep(1000);
                    target = 0;
                    pas++;
                }
            }

            if(pozitie == Pipeline_Red.SkystonePosition.RIGHT)
            {
                if(pas == 0){
                    DriveRight(1.23);
                    pas++;
                }

                if(pas == 1 && !motorFL.isBusy())
                {
                    DriveBackwards(1.8);
                    pas++;
                }

                if(pas == 2 && !motorFL.isBusy())
                {
                    DriveForward(0.4);
                    motorRoata.setPower(-0.8);
                    sleep(500);
                    motorRoata.setPower(0);
                    pas++;
                }

                if(pas == 3)
                {
                    stop_motors();
                    pas++;
                }
            }
        }
        webcam.stopStreaming();
    }
    public void stop_motors()
    {
        motorRL.setPower(0);
        motorRR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void DriveForward(double times)
    {
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorRL.setTargetPosition((int)(ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(0.5);
        motorRR.setPower(0.5);
        motorFL.setPower(0.5);
        motorFR.setPower(0.5);
    }

    public void DriveBackwards(double times)
    {
        motorRL.setTargetPosition((int)(-ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(-ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(-ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(-ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-0.5);
        motorRR.setPower(-0.5);
        motorFL.setPower(-0.5);
        motorFR.setPower(-0.5);
    }

    public void DriveLeft(double times)
    {
        motorRL.setTargetPosition((int)(ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(-ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(-ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int) (ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(0.5);
        motorRR.setPower(-0.5);
        motorFL.setPower(-0.5);
        motorFR.setPower(0.5);
    }
    public void DriveRight(double times)
    {
        motorRL.setTargetPosition((int)(-ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(-ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-0.5);
        motorRR.setPower(0.5);
        motorFL.setPower(0.5);
        motorFR.setPower(-0.5);
    }

    public void RotateLeft(double times)
    {
        motorRL.setTargetPosition((int) (-ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int) (ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int) (-ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int) (ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-1);
        motorRR.setPower(1);
        motorFL.setPower(-1);
        motorFR.setPower(1);
    }

    public void RotateRight(double times)
    {
        motorRL.setTargetPosition((int)(ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(-ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(-ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(1);
        motorRR.setPower(-1);
        motorFL.setPower(1);
        motorFR.setPower(-1);
    }
}
