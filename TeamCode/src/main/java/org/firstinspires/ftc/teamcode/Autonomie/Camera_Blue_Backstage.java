package org.firstinspires.ftc.teamcode.Autonomie;

import android.graphics.drawable.VectorDrawable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue_Detection_Backstage")
public class Camera_Blue_Backstage extends LinearOpMode{
    public static double kp = 0.001, ki, kd,kfst,kfdr, pos_servoin = 0, pos_servopus = 0.48, servo_error = 0.01,pos_servoBSuat = 0.20,aveon = 0;
    public static int target = 0,timerin,timersvin;
    public final double ticks_in_degree = 532/360.0;
    PIDController controller;
    Pipeline_Blue detection = new Pipeline_Blue();
    OpenCvWebcam webcam;
    final int ticks_motorFL = 536, ticks_motorFR = 529, ticks_motorRL = 538, ticks_motorRR = 525;
    DcMotor motorFL,motorFR, motorRL, motorRR, motorRoata, motorGL, motorGR;
    Servo servoBD, servoBS, servoin;
    boolean ok = true, carapalca = false;
    int pas = 0,timer=0,hampilarc = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(kp, ki, kd);
        motorFL = hardwareMap.get(DcMotor.class, "stsus");
        motorFR = hardwareMap.get(DcMotor.class, "drsus");
        motorRL = hardwareMap.get(DcMotor.class, "stjos");
        motorRR = hardwareMap.get(DcMotor.class, "drjos");
        motorGL = hardwareMap.get(DcMotor.class, "glis1");
        motorGR = hardwareMap.get(DcMotor.class, "glis2");
        motorRoata = hardwareMap.get(DcMotor.class, "motorin");

        servoBD = hardwareMap.get(Servo.class, "servodr");
        servoBS = hardwareMap.get(Servo.class, "servost");
        servoin = hardwareMap.get(Servo.class, "servoin");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRoata.setDirection(DcMotor.Direction.REVERSE);

        servoBD.setDirection(Servo.Direction.REVERSE);
        motorGL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorGL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int webcamID = hardwareMap.appContext.getResources().getIdentifier("webcamID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, webcamID);
        webcam.setPipeline(detection);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        Pipeline_Blue.SkystonePosition pozitie = detection.getAnalysis();

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
            double power = controller.calculate(motorGL.getCurrentPosition(), target);
            double power2 = controller.calculate(motorGR.getCurrentPosition(), target);
            double Kff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfst;
            double Kff2 = Math.cos(Math.toRadians(target / ticks_in_degree)) * kfdr;
            motorGL.setPower(power + Kff);
            motorGR.setPower(power2 + Kff2);

            if(motorGL.getCurrentPosition() > -2000)
            {
                servoBS.setPosition(pos_servoBSuat + servo_error);
                servoBD.setPosition(pos_servoBSuat);
            }
            if(motorGL.getCurrentPosition() < -1900)
            {
                servoBS.setPosition(pos_servopus + servo_error);
                servoBD.setPosition(pos_servopus);
            }
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            if(pozitie == Pipeline_Blue.SkystonePosition.LEFT && pas == 0)
            {
                TrajectorySequence TrajDr = drive.trajectorySequenceBuilder(new Pose2d())
                        .lineToConstantHeading(new Vector2d(28, 0))
                        .waitSeconds(0.5)
                        .lineToConstantHeading(new Vector2d(28, 15))
                        .lineToConstantHeading(new Vector2d(26, 22))
                        .lineToLinearHeading(new Pose2d(26.1,22,Math.toRadians(-90)))
                        .addDisplacementMarker(() -> {
                            motorRoata.setPower(-0.5);
                        })
                        .waitSeconds(1)
                        .lineToConstantHeading(new Vector2d(23, 22))
                        .addDisplacementMarker(() -> {
                            motorRoata.setPower(0);
                        })
                        .waitSeconds(1)
//                        .lineToConstantHeading(new Vector2d(23,40))
//                        .lineToLinearHeading(new Pose2d(23.1,40,Math.toRadians(-90)))
//                        .addDisplacementMarker(() -> {
//                            carapalca = true;
//                        })
                        .build();
                drive.followTrajectorySequence(TrajDr);
                pas ++;
            }
            else
            if(pozitie == Pipeline_Blue.SkystonePosition.CENTER && pas == 0 && !carapalca)
            {
                TrajectorySequence myTraj = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(30, 0))
                    .waitSeconds(0.5)
                    .lineToConstantHeading(new Vector2d(28, 0))
                    .addDisplacementMarker(() -> {
                        motorRoata.setPower(-0.5);
                    })
                    .waitSeconds(1)
                    .lineToConstantHeading(new Vector2d(25, 0))
                    .addDisplacementMarker(() -> {
                        motorRoata.setPower(0);
                    })
                    .waitSeconds(1)
//                    .lineToConstantHeading(new Vector2d(25,38))
//                    .lineToLinearHeading(new Pose2d(25.1,40,Math.toRadians(-90)))
//                        .addDisplacementMarker(() -> {
//                            carapalca = true;
//                                }
//                        )
                        .build();
                drive.followTrajectorySequence(myTraj);
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                pas++;
            }
            else
            if(pozitie == Pipeline_Blue.SkystonePosition.RIGHT && pas == 0)
            {
                TrajectorySequence TrajDr = drive.trajectorySequenceBuilder(new Pose2d())
                        .lineToConstantHeading(new Vector2d(28, 0))
                        .lineToConstantHeading(new Vector2d(28, -15))
                        .waitSeconds(0.5)
                        .lineToConstantHeading(new Vector2d(26, 0))
                        .lineToLinearHeading(new Pose2d(26.1,0,Math.toRadians(-90)))
                        .addDisplacementMarker(() -> {
                            motorRoata.setPower(-0.5);
                        })
                        .waitSeconds(1)
                        .lineToConstantHeading(new Vector2d(23, 0))
                        .addDisplacementMarker(() -> {
                            motorRoata.setPower(0);
                        })
                        .waitSeconds(1)
//                        .lineToConstantHeading(new Vector2d(23,40))
//                        .lineToLinearHeading(new Pose2d(23.1,40,Math.toRadians(-90)))
//                        .addDisplacementMarker(() -> {
//                            carapalca = true;
//                        })
                        .build();
                drive.followTrajectorySequence(TrajDr);
                pas++;
            }
            if(carapalca)
            {
                if(hampilarc < 100)
                {
                    target = -2700;
                    servoin.setPosition(0);
                }
                else servoin.setPosition(0.4);
                hampilarc++;
            }

            if(hampilarc > 200 && carapalca)
            {
                    target = 0;
            }
            if(hampilarc > 350 && carapalca)
            {
                TrajectorySequence mac = drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(22)
                        .back(5)
                        .build();
                drive.followTrajectorySequence(mac);
                carapalca = false;
            }
            Pose.pozo = motorGL.getCurrentPosition();
            telemetry.addData("hampilarc", hampilarc);
            telemetry.addData("carapalca", carapalca);
            telemetry.addData("servo", servoin.getPosition());
            telemetry.update();

        }

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